#include <Lidar/Roomscanner.h>
#include <math.h>
#include <iostream>
#include <chrono>
Roomscanner::Roomscanner(Lidar lidar_, std::vector<Room2D> rooms_) :
    lidar(lidar_), rooms(rooms_) {
  if (lidar.getSigma() < 1e-6) {
    add_lidar_noise = false;
    std::cout << "no lidar noise added " << std::endl;
  } else {
    add_lidar_noise = true;
    std::cout << "Lidar noise added " << lidar.getSigma() << std::endl;
    distribution_lidar = std::normal_distribution<double>(0., lidar.getSigma());
    std::random_device rd{};
    gen = std::mt19937{rd()};
  }
  // map is limited to 10m x 10m, with a resolution of 0.1 x 0.1 m we need 100 x 100 gridcells
  const int h = 200;
  const int w = 200;
  grid = Eigen::MatrixXi::Zero(h, w);
  cleaned_grid = Eigen::MatrixXi::Zero(h, w);
}

void Roomscanner::scanRoomsThread(const Vector3& state, const double I_theta_start,
     FullLidarMeasurement &ms, const int start_index, const int end_index) {
  Vector2 pos = state.topRows(2);
  
  double global_heading = I_theta_start;
  const int mps{lidar.getMeasurementsPerCycle()};

  for (int i = start_index; i <= end_index; i++) {
    // the rotation of the sensor for this measurement, i.e. at what angle
    // from the lidar's local coordinate frame the current ray is shot at
    double local_ray_angle = i * ((2 * PI) / mps);
    
    // the orientation of the ray shot in global coordinate frame, needed
    // because the room geometry is represented in the global frame
    double lidar_global_heading = i * ((2 * PI) / mps) + state(2);
    
    Point2 end_of_ray(lidar.getRange() * cos(lidar_global_heading),
                      lidar.getRange() * sin(lidar_global_heading));
    end_of_ray = end_of_ray + pos;                      
    Line ray(pos, end_of_ray);

    double min_dist = INFINITY;
    bool has_intersection = false;
    for (const Room2D& room : rooms) {
      const std::vector<Point2>& corners = room.getCorners();   
      for (int j=0; j < corners.size() - 1; j++) {
        Line wall(corners[j], corners[j+1]);
        double distance = computeDistanceRayToWall(ray, wall);
        if (distance >= 0 && distance < min_dist) {
          min_dist = distance;
          has_intersection = true;
        }
      }
    }
    // finally, add the measurement made. we store the measurements with
    // the lidar local angles
    if (has_intersection) {
      double noise = 0;
      if (add_lidar_noise) {
        noise = distribution_lidar(gen);
      }
      ms.measurements[i] = Measurement(min_dist + noise, local_ray_angle);
    } else {
      ms.measurements[i] = Measurement(-1, local_ray_angle);
    }
  }
}

void Roomscanner::scanRooms(const Vector3& state) {
  FullLidarMeasurement ms;
  ms.lidar_state = state;
  ms.measurements.clear();
  Vector2 pos = state.topRows(2);
  
  const int mps{lidar.getMeasurementsPerCycle()};
  ms.measurements.reserve(mps); // we know exactly how many measurements we make
  for (int i=0; i<mps; i++) {
    ms.measurements.push_back(Measurement(-1, 0.));
  }
  double global_heading = 0.;

  int nr_threads = 4;
  std::vector<std::thread> threads;
  const double dtheta = (2 * PI) / nr_threads;
  const int dindex = mps / nr_threads;
  for (int i = 0; i < nr_threads; i++) {
    threads.push_back(std::thread(&Roomscanner::scanRoomsThread, this, std::ref(state), i * dtheta,
    std::ref(ms), i * dindex, (i+1) * dindex - 1));
  }
  for (auto& t : threads) {
    t.join(); 
  }
  updateObservedGeometry(ms);
}

double Roomscanner::computeDistanceRayToWall(const Line& ray, const Line& wall) {
  // directional vector for ray & wall
  Vector2 ray_vector = ray.p2 - ray.p1;
  Vector2 wall_vector = wall.p2 - wall.p1;
  
  // check if the vectors are parallel, since in that case the method
  // may fail
  Vector2 v = ray_vector.normalized();
  Vector2 w = wall_vector.normalized();
  if (abs(v(0)) - abs(w(0)) < 1e-5 && abs(v(1)) - abs(w(1)) < 1e-5) {
    return -1;
  }
  
  Matrix2 A;
  A.col(0) = ray_vector;
  A.col(1) = -wall_vector;
  Vector2 b = wall.p1 - ray.p1; // right hand side of the LSE
  Vector2 x = A.colPivHouseholderQr().solve(b);
  
  // we have an intersection if the solution to above LSE is a vector
  // where both entries lie in [0,1]
  if (x(0) > 0. && x(0) <= 1. &&
      x(1) >= 0. && x(1) <= 1.) {
    // we now have to find the actual intersection point
    Point2 intersection_point = wall.p1 + x(1) * wall_vector;
    return (intersection_point - ray.p1).norm();
  } else {
    return -1.;
  }
}

void Roomscanner::mapMeasurementsToGrid(const FullLidarMeasurement& fms, int start_index,
      int end_index) {
  double theta = fms.lidar_state(2);
  double x = fms.lidar_state(0);
  double y = fms.lidar_state(1);
  
  Matrix3 T;
  T <<  cos(theta), -sin(theta), x,
        sin(theta),  cos(theta), y,
        0,           0,          1; 
  for (int i = start_index; i <= end_index; i++) {
    const Measurement& m = fms.measurements[i];
    if (m.distance >= 0) {
      Vector3 pt = T * Vector3(m.distance * cos(m.angle), 
                               m.distance * sin(m.angle), 1);
      // now we have to convert the point to the grid inidices
      int i = int(pt(0) * 10);
      int j = int(pt(1) * 10);
      if (i > grid.rows() || i < 0  ||
          j > grid.cols() || j < 0) {
        std::cout << "measured point outside of grid boundaries: " << pt.transpose() << std::endl;
      } else {
        grid(i,j) = grid(i,j) + 1;
      }
    }
  }
}

void Roomscanner::updateCleanedGrid(int start_i, int start_j, int sz) {
  for (int i = start_i; i <= start_i + sz; i++) {
    for (int j = start_j; j <= start_j + sz; j++) {
      int w_l = 1; int w_r = 1;
      int h_u = 1; int h_d = 1;

      if (i == 0) {
        h_u = 0;
      } else if (i == grid.rows() - 1) {
        h_d = 0;
      }
      if (j == 0) {
        w_l = 0;
      } else if (j == grid.cols() - 1) {
        w_r = 0;
      }

      int m = 1 + h_u + h_d;
      int n = 1 + w_l + w_r;
      double max = grid.block(i - h_u, j - w_l, m, n).maxCoeff();
      if (max > 0) {
        if (grid(i,j) / max > 0.75) {
          cleaned_grid(i,j) = 1;
        } else {
          cleaned_grid(i,j) = 0;
        }
      }
    }
  }
}

// TODO this is very inefficient, find a better way to do this
// or try to accelerate by parallelizing
void Roomscanner::updateObservedGeometry(const FullLidarMeasurement& fms) {
  
  // TODO maybe revert since this is not the bottleneck
  int nr_threads = 1;
  int mps = lidar.getMeasurementsPerCycle();
  int nr_ms_per_thread = mps / nr_threads;

  std::vector<std::thread> threads;
  for (int i = 0; i < nr_threads; i++) {
    threads.push_back(std::thread(&Roomscanner::mapMeasurementsToGrid, this, std::ref(fms),
          i * nr_ms_per_thread, (i+1) * nr_ms_per_thread - 1));
  }
  for (auto& t : threads) {
    t.join();
  }

  threads.clear();
  nr_threads = 4;
  const int row_blocks = 2;
  const int col_blocks = 2;
  const int sz = grid.rows() / 4; // assuming a square grid
  for (int i = 0; i < row_blocks; i++) {
    for (int j = 0; j < col_blocks; j++) {
      threads.push_back(std::thread(&Roomscanner::updateCleanedGrid,
      this, i * sz, j * sz, sz));
    }
  }
  for (auto& t : threads) {
    t.join();
  }
}