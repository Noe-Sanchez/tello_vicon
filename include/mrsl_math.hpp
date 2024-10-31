#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

double sig(double x, double exponent){
  double s;

  if (x > 0){
    s = 1;
  } else if (x < 0){
    s = -1;
  } else {
    s = 0;
  }

  s = pow(abs(x), exponent) * s;

  return s;
}

template <typename VectorType>
VectorType sig(VectorType v, double exponent){
  VectorType s;

  for (int i = 0; i < v.size(); i++){
    if (v(i) > 0){
      s(i) = 1;
    } else if (v(i) < 0){
      s(i) = -1;
    } else {
      s(i) = 0;
    }

    s(i) = pow(abs(v(i)), exponent) * s(i);
  }

  return s;
}

template <typename VectorType>
VectorType sig(VectorType v, VectorType exponent){
  VectorType s;

  for (int i = 0; i < v.size(); i++){
    if (v(i) > 0){
      s(i) = 1;
    } else if (v(i) < 0){
      s(i) = -1;
    } else {
      s(i) = 0;
    }

    s(i) = pow(abs(v(i)), exponent(i)) * s(i);
  }

  return s;
}

Eigen::Vector4d ewise(Eigen::Vector4d v1, Eigen::Vector4d v2){
  Eigen::Vector4d v;

  for (int i = 0; i < 4; i++){
    v(i) = v1(i) * v2(i);
  }

  return v;
}

Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::Vector4d p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  q_matrix << q(0), -q(1), -q(2), -q(3),
              q(1),  q(0), -q(3),  q(2),
	      q(2),  q(3),  q(0), -q(1),
	      q(3), -q(2),  q(1),  q(0);

  p_vector = q_matrix * p;

  return p_vector;
}

Eigen::Vector4d kronecker(Eigen::Vector4d q, Eigen::Vector3d p){
  Eigen::Matrix4d q_matrix;
  Eigen::Vector4d p_vector;

  p_vector << 0, p(0), p(1), p(2);

  q_matrix << q(0), -q(1), -q(2), -q(3),
	            q(1),  q(0), -q(3),  q(2),
	            q(2),  q(3),  q(0), -q(1),
	            q(3), -q(2),  q(1),  q(0);
	      
  p_vector = q_matrix * p_vector;
  
  return p_vector;
}

Eigen::Vector4d qlm(Eigen::Vector4d q){
  Eigen::Vector4d q_log;

  //double norm = sqrt(q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
  double norm = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3));

  if (norm < 0.01){
    q_log << 0, 0, 0, 0;
  } else {
    // Use arccos to get the angle
    q_log << 0, q(1)/norm, q(2)/norm, q(3)/norm;
    if (q(0) > 1){
      q(0) = 1;
    } else if (q(0) < -1){
      q(0) = -1;
    }
    double acosaux = acos(q(0)) < acos(-q(0)) ? acos(q(0)) : acos(-q(0));
    //q_log = acos(q(0)) * q_log;
    q_log = acosaux * q_log;
  }

  return 2*q_log;
}

Eigen::Vector4d exp4(Eigen::Vector4d v, double exponent){
  Eigen::Vector4d v_exp;

  for (int i = 0; i < 4; i++){
    v_exp(i) = pow(v(i), exponent);
  }

  return v_exp;
}

Eigen::Vector4d sqrt4(Eigen::Vector4d v){
  Eigen::Vector4d s;

  for (int i = 0; i < 4; i++){
    s(i) = sqrt(v(i));
  }

  return s;
}
