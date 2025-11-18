const mr = require('./modern_robotics_core.js');

/**
 * Calculate relative rotation matrix between current and initial rotation
 * @param {Array<Array<number>>} currentR - Current rotation matrix (3x3)
 * @param {Array<Array<number>>} initialR - Initial rotation matrix (3x3)
 * @param {string} mode - Calculation mode: 'inSpace' or 'inBody'
 * @returns {Array<Array<number>>} - Relative rotation matrix (3x3)
 */
function calculateRelativeRotationMatrix (currentR, initialR, mode) {
  if (!initialR) {
    // If no initial matrix, return identity matrix
    return [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ];
  }

  // Calculate transpose of initial rotation matrix
  const initialR_T = [
    [initialR[0][0], initialR[1][0], initialR[2][0]],
    [initialR[0][1], initialR[1][1], initialR[2][1]],
    [initialR[0][2], initialR[1][2], initialR[2][2]]
  ];

  const relativeR = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]
  ];

  if (mode === 'inSpace') {
    // Calculate R_relative = R_current * R_initial^T (Space frame reference)
    // Matrix multiplication: R_current * R_initial^T
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        relativeR[i][j] = 
          currentR[i][0] * initialR_T[0][j] +
          currentR[i][1] * initialR_T[1][j] +
          currentR[i][2] * initialR_T[2][j];
      }
    }
  } else if (mode === 'inBody') {
    // Calculate R_relative = R_initial^T * R_current (Body frame reference)
    // Matrix multiplication: R_initial^T * R_current
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        relativeR[i][j] = 
          initialR_T[i][0] * currentR[0][j] +
          initialR_T[i][1] * currentR[1][j] +
          initialR_T[i][2] * currentR[2][j];
      }
    }
  } else {
    throw new Error(`Invalid mode: ${mode}. Use 'inSpace' or 'inBody'.`);
  }

  return relativeR;
}


/**
 * Convert rotation matrix to screw axis representation
 * @param {Array<Array<number>>} relativeR - Relative rotation matrix (3x3)
 * @returns {Array} - [omega_hat, theta] where omega_hat is unit axis vector and theta is rotation angle
 */
function relativeRMatrixtoScrewAxis(relativeR) {
    // Calculate so(3) matrix using MatrixLog3
    const so3mat = mr.MatrixLog3(relativeR);
    
    // Extract omega_theta vector from so(3) matrix
    const omega_theta = mr.so3ToVec(so3mat);
    
    // Calculate magnitude (theta) using JavaScript array operations
    const theta = Math.sqrt(
        omega_theta[0] * omega_theta[0] + 
        omega_theta[1] * omega_theta[1] + 
        omega_theta[2] * omega_theta[2]
    );
    
    let omega_hat;
    
    if (theta < 1e-6) {
        // If theta is very small, return zero vector
        omega_hat = [0, 0, 0];
    } else {
        // Normalize omega_theta to get unit axis vector
        omega_hat = [
            omega_theta[0] / theta,
            omega_theta[1] / theta,
            omega_theta[2] / theta
        ];
    }
    
    return [omega_hat, theta];
}

/**
 * Convert screw axis and angle to relative rotation matrix
 * @param {Array<number>} omega_hat - Unit axis vector [x, y, z]
 * @param {number} theta - Rotation angle in radians
 * @returns {Array<Array<number>>} - Relative rotation matrix (3x3)
 */
function ScrewAxisToRelativeRMatrix(omega_hat, theta) {
    // Create omega_theta vector by multiplying unit axis with angle
    const omega_theta = [
        omega_hat[0] * theta,
        omega_hat[1] * theta,
        omega_hat[2] * theta
    ];
    
    // Convert omega_theta vector to so(3) matrix
    const so3mat = mr.VecToso3(omega_theta);
    // Convert so(3) matrix to rotation matrix using MatrixExp3
    const R_relative = mr.MatrixExp3(so3mat);
    
    return R_relative;
}

function calculateSpatialVelocity(T_current, T_target, dt, mode) {
  // T_sd^(-1) * T_target
  const T_current_inv = mr.TransInv(T_current);
  let relative_T;
  if (mode === 'inBody') {
    // Calculate relative transform T_sd^(-1) * T_target
    relative_T = mr.matDot(T_current_inv, T_target);
  } else if (mode === 'inSpace') {
    // Calculate relative transform T_target * T_sd^(-1)
    relative_T = mr.matDot(T_target, T_current_inv);
  } else {
    throw new Error(`Invalid mode: ${mode}. Use 'inSpace' or 'inBody'.`);
  }
  
  // Calculate matrix logarithm (Lie algebra)
  const se3_matrix = mr.MatrixLog6(relative_T);
  
  // Extract 6D vector from skew-symmetric matrix
  const V_d_raw = mr.se3ToVec(se3_matrix);
  
  // Scale to actual velocity (divide by time step)
  const V_d = V_d_raw.map(v => v / dt);
  
  return V_d;
}

module.exports = {
    calculateRelativeRotationMatrix,
    relativeRMatrixtoScrewAxis,
    ScrewAxisToRelativeRMatrix,
    calculateSpatialVelocity
};