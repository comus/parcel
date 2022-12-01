/******/ (() => { // webpackBootstrap
/******/ 	var __webpack_modules__ = ({

/***/ "./mindar/image-target/estimation/estimate.js":
/*!****************************************************!*\
  !*** ./mindar/image-target/estimation/estimate.js ***!
  \****************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {Matrix, inverse} = __webpack_require__(/*! ml-matrix */ "./node_modules/ml-matrix/src/index.js");
const {solveHomography} = __webpack_require__(/*! ../utils/homography */ "./mindar/image-target/utils/homography.js");

// build world matrix with list of matching worldCoords|screenCoords
//
// Step 1. estimate homography with list of pairs
// Ref: https://www.uio.no/studier/emner/matnat/its/TEK5030/v19/lect/lecture_4_3-estimating-homographies-from-feature-correspondences.pdf  (Basic homography estimation from points)
//
// Step 2. decompose homography into rotation and translation matrixes (i.e. world matrix)
// Ref: can anyone provide reference?
const estimate = ({screenCoords, worldCoords, projectionTransform}) => {
  const Harray = solveHomography(worldCoords.map((p) => [p.x, p.y]), screenCoords.map((p) => [p.x, p.y]));
  const H = new Matrix([
    [Harray[0], Harray[1], Harray[2]],
    [Harray[3], Harray[4], Harray[5]],
    [Harray[6], Harray[7], Harray[8]],
  ]);

  const K = new Matrix(projectionTransform);
  const KInv = inverse(K);

  const _KInvH = KInv.mmul(H);
  const KInvH = _KInvH.to1DArray();

  const norm1 = Math.sqrt( KInvH[0] * KInvH[0] + KInvH[3] * KInvH[3] + KInvH[6] * KInvH[6]);
  const norm2 = Math.sqrt( KInvH[1] * KInvH[1] + KInvH[4] * KInvH[4] + KInvH[7] * KInvH[7]);
  const tnorm = (norm1 + norm2) / 2;

  const rotate = [];
  rotate[0] = KInvH[0] / norm1;
  rotate[3] = KInvH[3] / norm1;
  rotate[6] = KInvH[6] / norm1;

  rotate[1] = KInvH[1] / norm2;
  rotate[4] = KInvH[4] / norm2;
  rotate[7] = KInvH[7] / norm2;

  rotate[2] = rotate[3] * rotate[7] - rotate[6] * rotate[4];
  rotate[5] = rotate[6] * rotate[1] - rotate[0] * rotate[7];
  rotate[8] = rotate[0] * rotate[4] - rotate[1] * rotate[3];

  const norm3 = Math.sqrt(rotate[2] * rotate[2] + rotate[5] * rotate[5] + rotate[8] * rotate[8]);
  rotate[2] /= norm3;
  rotate[5] /= norm3;
  rotate[8] /= norm3;

  // TODO: artoolkit has check_rotation() that somehow switch the rotate vector. not sure what that does. Can anyone advice?
  // https://github.com/artoolkitx/artoolkit5/blob/5bf0b671ff16ead527b9b892e6aeb1a2771f97be/lib/SRC/ARICP/icpUtil.c#L215

  const tran = []
  tran[0] = KInvH[2] / tnorm;
  tran[1] = KInvH[5] / tnorm;
  tran[2] = KInvH[8] / tnorm;

  let initialModelViewTransform = [
    [rotate[0], rotate[1], rotate[2], tran[0]],
    [rotate[3], rotate[4], rotate[5], tran[1]],
    [rotate[6], rotate[7], rotate[8], tran[2]]
  ];

  return initialModelViewTransform;
};

module.exports = {
  estimate
}


/***/ }),

/***/ "./mindar/image-target/estimation/estimator.js":
/*!*****************************************************!*\
  !*** ./mindar/image-target/estimation/estimator.js ***!
  \*****************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {estimate} = __webpack_require__(/*! ./estimate.js */ "./mindar/image-target/estimation/estimate.js");
const {refineEstimate} = __webpack_require__(/*! ./refine-estimate.js */ "./mindar/image-target/estimation/refine-estimate.js");

class Estimator {
  constructor(projectionTransform) {
    this.projectionTransform = projectionTransform;
  }

  // Solve homography between screen points and world points using Direct Linear Transformation
  // then decompose homography into rotation and translation matrix (i.e. modelViewTransform)
  estimate({screenCoords, worldCoords}) {
    const modelViewTransform = estimate({screenCoords, worldCoords, projectionTransform: this.projectionTransform});
    return modelViewTransform;
  }

  // Given an initial guess of the modelViewTransform and new pairs of screen-world coordinates, 
  // use Iterative Closest Point to refine the transformation
  //refineEstimate({initialModelViewTransform, screenCoords, worldCoords}) {
  refineEstimate({initialModelViewTransform, worldCoords, screenCoords}) {
    const updatedModelViewTransform = refineEstimate({initialModelViewTransform, worldCoords, screenCoords, projectionTransform: this.projectionTransform});
    return updatedModelViewTransform;
  }
}

module.exports = {
  Estimator,
}


/***/ }),

/***/ "./mindar/image-target/estimation/refine-estimate.js":
/*!***********************************************************!*\
  !*** ./mindar/image-target/estimation/refine-estimate.js ***!
  \***********************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {Matrix, inverse} = __webpack_require__(/*! ml-matrix */ "./node_modules/ml-matrix/src/index.js");
const {normalizePoints, applyModelViewProjectionTransform, buildModelViewProjectionTransform, computeScreenCoordiate} = __webpack_require__(/*! ./utils.js */ "./mindar/image-target/estimation/utils.js");

const TRACKING_THRESH = 5.0; // default
const K2_FACTOR = 4.0; // Question: should it be relative to the size of the screen instead of hardcoded?
const ICP_MAX_LOOP = 10;
const ICP_BREAK_LOOP_ERROR_THRESH = 0.1;
const ICP_BREAK_LOOP_ERROR_RATIO_THRESH = 0.99;
const ICP_BREAK_LOOP_ERROR_THRESH2 = 4.0;

// some temporary/intermediate variables used later. Declare them beforehand to reduce new object allocations
let mat = [[],[],[]]; 
let J_U_Xc = [[],[]]; // 2x3
let J_Xc_S = [[],[],[]]; // 3x6

const refineEstimate = ({initialModelViewTransform, projectionTransform, worldCoords, screenCoords}) => {
  // Question: shall we normlize the screen coords as well?
  // Question: do we need to normlize the scale as well, i.e. make coords from -1 to 1
  //
  // normalize world coords - reposition them to center of mass
  //   assume z coordinate is always zero (in our case, the image target is planar with z = 0
  let dx = 0;
  let dy = 0;
  for (let i = 0; i < worldCoords.length; i++) {
    dx += worldCoords[i].x;
    dy += worldCoords[i].y;
  }
  dx /= worldCoords.length;
  dy /= worldCoords.length;

  const normalizedWorldCoords = [];
  for (let i = 0; i < worldCoords.length; i++) {
    normalizedWorldCoords.push({x: worldCoords[i].x - dx, y: worldCoords[i].y - dy, z: worldCoords[i].z});
  }

  const diffModelViewTransform = [[],[],[]];
  for (let j = 0; j < 3; j++) {
    for (let i = 0; i < 3; i++) {
      diffModelViewTransform[j][i] = initialModelViewTransform[j][i];
    }
  }
  diffModelViewTransform[0][3] = initialModelViewTransform[0][0] * dx + initialModelViewTransform[0][1] * dy + initialModelViewTransform[0][3];
  diffModelViewTransform[1][3] = initialModelViewTransform[1][0] * dx + initialModelViewTransform[1][1] * dy + initialModelViewTransform[1][3];
  diffModelViewTransform[2][3] = initialModelViewTransform[2][0] * dx + initialModelViewTransform[2][1] * dy + initialModelViewTransform[2][3];

  // use iterative closest point algorithm to refine the modelViewTransform
  const inlierProbs = [1.0, 0.8, 0.6, 0.4, 0.0];
  let updatedModelViewTransform = diffModelViewTransform; // iteratively update this transform
  let finalModelViewTransform = null;
  for (let i = 0; i < inlierProbs.length; i++) {
    const ret = _doICP({initialModelViewTransform: updatedModelViewTransform, projectionTransform, worldCoords: normalizedWorldCoords, screenCoords, inlierProb: inlierProbs[i]});

    updatedModelViewTransform = ret.modelViewTransform;

    //console.log("err", ret.err);

    if (ret.err < TRACKING_THRESH) {
      finalModelViewTransform = updatedModelViewTransform;
      break;
    }
  }

  if (finalModelViewTransform === null) return null;

  // de-normalize
  finalModelViewTransform[0][3] = finalModelViewTransform[0][3] - finalModelViewTransform[0][0] * dx - finalModelViewTransform[0][1] * dy;
  finalModelViewTransform[1][3] = finalModelViewTransform[1][3] - finalModelViewTransform[1][0] * dx - finalModelViewTransform[1][1] * dy;
  finalModelViewTransform[2][3] = finalModelViewTransform[2][3] - finalModelViewTransform[2][0] * dx - finalModelViewTransform[2][1] * dy;

  return finalModelViewTransform;
}

// ICP iteration
// Question: can someone provide theoretical reference / mathematical proof for the following computations?
const _doICP = ({initialModelViewTransform, projectionTransform, worldCoords, screenCoords, inlierProb}) => {
  const isRobustMode = inlierProb < 1;

  let modelViewTransform = initialModelViewTransform;

  let err0 = 0.0;
  let err1 = 0.0;

  let E = new Array(worldCoords.length);
  let E2 = new Array(worldCoords.length);
  let dxs = new Array(worldCoords.length);
  let dys = new Array(worldCoords.length);

  for (let l = 0; l <= ICP_MAX_LOOP; l++) {
    const modelViewProjectionTransform = buildModelViewProjectionTransform(projectionTransform, modelViewTransform);

    for (let n = 0; n < worldCoords.length; n++) {
      const u = computeScreenCoordiate(modelViewProjectionTransform, worldCoords[n].x, worldCoords[n].y, worldCoords[n].z);
      const dx = screenCoords[n].x - u.x;
      const dy = screenCoords[n].y - u.y;

      dxs[n] = dx;
      dys[n] = dy;
      E[n] = (dx * dx + dy * dy);
    }

    let K2; // robust mode only
    err1 = 0.0;
    if (isRobustMode) {
      const inlierNum = Math.max(3, Math.floor(worldCoords.length * inlierProb) - 1);
      for (let n = 0; n < worldCoords.length; n++) {
        E2[n] = E[n];
      }
      E2.sort((a, b) => {return a-b;});

      K2 = Math.max(E2[inlierNum] * K2_FACTOR, 16.0);
      for (let n = 0; n < worldCoords.length; n++) {
        if (E2[n] > K2) err1 += K2/ 6;
        else err1 +=  K2/6.0 * (1.0 - (1.0-E2[n]/K2)*(1.0-E2[n]/K2)*(1.0-E2[n]/K2));
      }
    } else {
      for (let n = 0; n < worldCoords.length; n++) {
        err1 += E[n];
      }
    }
    err1 /= worldCoords.length;

    //console.log("icp loop", inlierProb, l, err1);

    if (err1 < ICP_BREAK_LOOP_ERROR_THRESH) break;
    //if (l > 0 && err1 < ICP_BREAK_LOOP_ERROR_THRESH2 && err1/err0 > ICP_BREAK_LOOP_ERROR_RATIO_THRESH) break;
    if (l > 0 && err1/err0 > ICP_BREAK_LOOP_ERROR_RATIO_THRESH) break;
    if (l === ICP_MAX_LOOP) break;

    err0 = err1;

    const dU = [];
    const allJ_U_S = [];
    for (let n = 0; n < worldCoords.length; n++) {
      if (isRobustMode && E[n] > K2) {
        continue;
      }

      const J_U_S = _getJ_U_S({modelViewProjectionTransform, modelViewTransform, projectionTransform, worldCoord: worldCoords[n]});

      if (isRobustMode) {
        const W = (1.0 - E[n]/K2)*(1.0 - E[n]/K2);

        for (let j = 0; j < 2; j++) {
          for (let i = 0; i < 6; i++) {
            J_U_S[j][i] *= W;
          }
        }
        dU.push([dxs[n] * W]);
        dU.push([dys[n] * W]);
      } else {
        dU.push([dxs[n]]);
        dU.push([dys[n]]);
      }

      for (let i = 0; i < J_U_S.length; i++) {
        allJ_U_S.push(J_U_S[i]);
      }
    }

    const dS = _getDeltaS({dU, J_U_S: allJ_U_S});
    if (dS === null) break;

    modelViewTransform = _updateModelViewTransform({modelViewTransform, dS});
  }
  return {modelViewTransform, err: err1};
}

const _updateModelViewTransform = ({modelViewTransform, dS}) => {
  /**
   * dS has 6 paragrams, first half is rotation, second half is translation
   * rotation is expressed in angle-axis, 
   *   [S[0], S[1] ,S[2]] is the axis of rotation, and the magnitude is the angle
   */
  let ra = dS[0] * dS[0] + dS[1] * dS[1] + dS[2] * dS[2];
  let q0, q1, q2;
  if( ra < 0.000001 ) {
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    ra = 0.0;
  } else {
    ra = Math.sqrt(ra);
    q0 = dS[0] / ra;
    q1 = dS[1] / ra;
    q2 = dS[2] / ra;
  }

  const cra = Math.cos(ra);
  const sra = Math.sin(ra);
  const one_cra = 1.0 - cra;

  // mat is [R|t], 3D rotation and translation
  mat[0][0] = q0*q0*one_cra + cra;
  mat[0][1] = q0*q1*one_cra - q2*sra;
  mat[0][2] = q0*q2*one_cra + q1*sra;
  mat[0][3] = dS[3];
  mat[1][0] = q1*q0*one_cra + q2*sra;
  mat[1][1] = q1*q1*one_cra + cra;
  mat[1][2] = q1*q2*one_cra - q0*sra;
  mat[1][3] = dS[4]
  mat[2][0] = q2*q0*one_cra - q1*sra;
  mat[2][1] = q2*q1*one_cra + q0*sra;
  mat[2][2] = q2*q2*one_cra + cra;
  mat[2][3] = dS[5];

  // the updated transform is the original transform x delta transform
  const mat2 = [[],[],[]];
  for (let j = 0; j < 3; j++ ) {
    for (let i = 0; i < 4; i++ ) {
      mat2[j][i] = modelViewTransform[j][0] * mat[0][i]
                   + modelViewTransform[j][1] * mat[1][i]
                   + modelViewTransform[j][2] * mat[2][i];
    }
    mat2[j][3] += modelViewTransform[j][3];
  }
  return mat2;
}

const _getDeltaS = ({dU, J_U_S}) => {
  const J = new Matrix(J_U_S);
  const U = new Matrix(dU);

  const JT = J.transpose();
  const JTJ = JT.mmul(J);
  const JTU = JT.mmul(U);

  let JTJInv;
  try {
    JTJInv = inverse(JTJ);
  } catch (e) {
    return null;
  }

  const S = JTJInv.mmul(JTU);
  return S.to1DArray();
}

const _getJ_U_S = ({modelViewProjectionTransform, modelViewTransform, projectionTransform, worldCoord}) => {
  const T = modelViewTransform;
  const {x, y, z} = worldCoord;

  const u = applyModelViewProjectionTransform(modelViewProjectionTransform, x, y, z);

  const z2 = u.z * u.z;
  // Question: This is the most confusing matrix to me. I've no idea how to derive this.
  //J_U_Xc[0][0] = (projectionTransform[0][0] * u.z - projectionTransform[2][0] * u.x) / z2;
  //J_U_Xc[0][1] = (projectionTransform[0][1] * u.z - projectionTransform[2][1] * u.x) / z2;
  //J_U_Xc[0][2] = (projectionTransform[0][2] * u.z - projectionTransform[2][2] * u.x) / z2;
  //J_U_Xc[1][0] = (projectionTransform[1][0] * u.z - projectionTransform[2][0] * u.y) / z2;
  //J_U_Xc[1][1] = (projectionTransform[1][1] * u.z - projectionTransform[2][1] * u.y) / z2;
  //J_U_Xc[1][2] = (projectionTransform[1][2] * u.z - projectionTransform[2][2] * u.y) / z2;
  
  // The above is the original implementation, but simplify to below becuase projetionTransform[2][0] and [2][1] are zero
  J_U_Xc[0][0] = (projectionTransform[0][0] * u.z) / z2;
  J_U_Xc[0][1] = (projectionTransform[0][1] * u.z) / z2;
  J_U_Xc[0][2] = (projectionTransform[0][2] * u.z - projectionTransform[2][2] * u.x) / z2;
  J_U_Xc[1][0] = (projectionTransform[1][0] * u.z) / z2;
  J_U_Xc[1][1] = (projectionTransform[1][1] * u.z) / z2;
  J_U_Xc[1][2] = (projectionTransform[1][2] * u.z - projectionTransform[2][2] * u.y) / z2;

  /*
    J_Xc_S should be like this, but z is zero, so we can simplify
    [T[0][2] * y - T[0][1] * z, T[0][0] * z - T[0][2] * x, T[0][1] * x - T[0][0] * y, T[0][0], T[0][1], T[0][2]],
    [T[1][2] * y - T[1][1] * z, T[1][0] * z - T[1][2] * x, T[1][1] * x - T[1][0] * y, T[1][0], T[1][1], T[1][2]],
    [T[2][2] * y - T[2][1] * z, T[2][0] * z - T[2][2] * x, T[2][1] * x - T[2][0] * y, T[2][0], T[2][1], T[2][2]],
  */
  J_Xc_S[0][0] = T[0][2] * y;
  J_Xc_S[0][1] = -T[0][2] * x;
  J_Xc_S[0][2] = T[0][1] * x - T[0][0] * y;
  J_Xc_S[0][3] = T[0][0];
  J_Xc_S[0][4] = T[0][1]; 
  J_Xc_S[0][5] = T[0][2];

  J_Xc_S[1][0] = T[1][2] * y;
  J_Xc_S[1][1] = -T[1][2] * x;
  J_Xc_S[1][2] = T[1][1] * x - T[1][0] * y;
  J_Xc_S[1][3] = T[1][0];
  J_Xc_S[1][4] = T[1][1];
  J_Xc_S[1][5] = T[1][2];

  J_Xc_S[2][0] = T[2][2] * y;
  J_Xc_S[2][1] = -T[2][2] * x;
  J_Xc_S[2][2] = T[2][1] * x - T[2][0] * y;
  J_Xc_S[2][3] = T[2][0];
  J_Xc_S[2][4] = T[2][1];
  J_Xc_S[2][5] = T[2][2];

  const J_U_S = [[], []];
  for (let j = 0; j < 2; j++) {
    for (let i = 0; i < 6; i++) {
      J_U_S[j][i] = 0.0;
      for (let k = 0; k < 3; k++ ) {
        J_U_S[j][i] += J_U_Xc[j][k] * J_Xc_S[k][i];
      }
    }
  }
  return J_U_S;
}

module.exports = {
  refineEstimate
}


/***/ }),

/***/ "./mindar/image-target/estimation/utils.js":
/*!*************************************************!*\
  !*** ./mindar/image-target/estimation/utils.js ***!
  \*************************************************/
/***/ ((module) => {

const buildModelViewProjectionTransform = (projectionTransform, modelViewTransform) => {
  // assume the projectTransform has the following format:
  // [[fx, 0, cx],
  //  [0, fy, cy]
  //  [0, 0, 1]]
  const modelViewProjectionTransform = [
    [
      projectionTransform[0][0] * modelViewTransform[0][0] + projectionTransform[0][2] * modelViewTransform[2][0],
      projectionTransform[0][0] * modelViewTransform[0][1] + projectionTransform[0][2] * modelViewTransform[2][1],
      projectionTransform[0][0] * modelViewTransform[0][2] + projectionTransform[0][2] * modelViewTransform[2][2],
      projectionTransform[0][0] * modelViewTransform[0][3] + projectionTransform[0][2] * modelViewTransform[2][3],
    ],
    [
      projectionTransform[1][1] * modelViewTransform[1][0] + projectionTransform[1][2] * modelViewTransform[2][0],
      projectionTransform[1][1] * modelViewTransform[1][1] + projectionTransform[1][2] * modelViewTransform[2][1],
      projectionTransform[1][1] * modelViewTransform[1][2] + projectionTransform[1][2] * modelViewTransform[2][2],
      projectionTransform[1][1] * modelViewTransform[1][3] + projectionTransform[1][2] * modelViewTransform[2][3],
    ],
    [
      modelViewTransform[2][0],
      modelViewTransform[2][1],
      modelViewTransform[2][2],
      modelViewTransform[2][3],
    ]
  ];
  return modelViewProjectionTransform;
  
  /*
  // this is the full computation if the projectTransform does not look like the expected format, but more computations
  //  
  const modelViewProjectionTransform = [[],[],[]];
  for (let j = 0; j < 3; j++ ) {
    for (let i = 0; i < 4; i++) {
      modelViewProjectionTransform[j][i] = projectionTransform[j][0] * modelViewTransform[0][i]
                                         + projectionTransform[j][1] * modelViewTransform[1][i]
                                         + projectionTransform[j][2] * modelViewTransform[2][i];
    }
  }
  return modelViewProjectionTransform;
  */
}

const applyModelViewProjectionTransform = (modelViewProjectionTransform, x, y, z) => {
  // assume z is zero
  const ux = modelViewProjectionTransform[0][0] * x + modelViewProjectionTransform[0][1] * y + modelViewProjectionTransform[0][3];
  const uy = modelViewProjectionTransform[1][0] * x + modelViewProjectionTransform[1][1] * y + modelViewProjectionTransform[1][3];
  const uz = modelViewProjectionTransform[2][0] * x + modelViewProjectionTransform[2][1] * y + modelViewProjectionTransform[2][3];
  return {x: ux, y: uy, z: uz};
}

const computeScreenCoordiate = (modelViewProjectionTransform, x, y, z) => {
  const {x: ux, y: uy, z: uz} = applyModelViewProjectionTransform(modelViewProjectionTransform, x, y, z);
  //if( Math.abs(uz) < 0.000001 ) return null;
  return {x: ux/uz, y: uy/uz};
}

const screenToMarkerCoordinate = (modelViewProjectionTransform, sx, sy) => {
  const c11 = modelViewProjectionTransform[2][0] * sx - modelViewProjectionTransform[0][0];
  const c12 = modelViewProjectionTransform[2][1] * sx - modelViewProjectionTransform[0][1];
  const c21 = modelViewProjectionTransform[2][0] * sy - modelViewProjectionTransform[1][0];
  const c22 = modelViewProjectionTransform[2][1] * sy - modelViewProjectionTransform[1][1];
  const b1  = modelViewProjectionTransform[0][3] - modelViewProjectionTransform[2][3] * sx;
  const b2  = modelViewProjectionTransform[1][3] - modelViewProjectionTransform[2][3] * sy;

  const m = c11 * c22 - c12 * c21;
  return {
    x: (c22 * b1 - c12 * b2) / m,
    y: (c11 * b2 - c21 * b1) / m
  }
}

module.exports = {
  buildModelViewProjectionTransform,
  applyModelViewProjectionTransform,
  computeScreenCoordiate,
}


/***/ }),

/***/ "./mindar/image-target/matching/hamming-distance.js":
/*!**********************************************************!*\
  !*** ./mindar/image-target/matching/hamming-distance.js ***!
  \**********************************************************/
/***/ ((module) => {

// Fast computation on number of bit sets
// Ref: https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
const compute = (options) => {
  const {v1, v2} = options;
  let d = 0;

  for (let i = 0; i < v1.length; i++) {
    let x = (v1[i] ^ v2[i]) >>> 0;
    d += bitCount(x);
  }
  return d;
}

const bitCount = (v) => {
  var c = v - ((v >> 1) & 0x55555555);
  c = ((c >> 2) & 0x33333333) + (c & 0x33333333);
  c = ((c >> 4) + c) & 0x0F0F0F0F;
  c = ((c >> 8) + c) & 0x00FF00FF;
  c = ((c >> 16) + c) & 0x0000FFFF;
  return c;
}

module.exports = {
  compute
};


/***/ }),

/***/ "./mindar/image-target/matching/hough.js":
/*!***********************************************!*\
  !*** ./mindar/image-target/matching/hough.js ***!
  \***********************************************/
/***/ ((module) => {

const kHoughBinDelta = 1;

// mathces [querypointIndex:x, keypointIndex: x]
const computeHoughMatches = (options) => {
  const {keywidth, keyheight, querywidth, queryheight, matches} = options;

  const maxX = querywidth * 1.2;
  const minX = -maxX;
  const maxY = queryheight * 1.2;
  const minY = -maxY;
  const numAngleBins = 12;
  const numScaleBins = 10;
  const minScale = -1;
  const maxScale = 1;
  const scaleK = 10.0;
  const scaleOneOverLogK = 1.0 / Math.log(scaleK);
  const maxDim = Math.max(keywidth, keyheight);
  const keycenterX = Math.floor(keywidth / 2);
  const keycenterY = Math.floor(keyheight / 2);

  // compute numXBins and numYBins based on matches
  const projectedDims = [];
  for (let i = 0; i < matches.length; i++) {
    const queryscale = matches[i].querypoint.scale;
    const keyscale = matches[i].keypoint.scale;
    if (keyscale == 0) console.log("ERROR divide zero");
    const scale = queryscale / keyscale;
    projectedDims.push( scale * maxDim );
  }

  // TODO optimize median
  //   weird. median should be [Math.floor(projectedDims.length/2) - 1] ?
  projectedDims.sort((a1, a2) => {return a1 - a2});
  const medianProjectedDim = projectedDims[ Math.floor(projectedDims.length/2) - (projectedDims.length%2==0?1:0) -1 ];

  const binSize = 0.25 * medianProjectedDim;
  const numXBins = Math.max(5, Math.ceil((maxX - minX) / binSize));
  const numYBins = Math.max(5, Math.ceil((maxY - minY) / binSize));

  const numXYBins = numXBins * numYBins;
  const numXYAngleBins = numXYBins * numAngleBins;

  // do voting
  const querypointValids = [];
  const querypointBinLocations = [];
  const votes = {};
  for (let i = 0; i < matches.length; i++) {
    const querypoint = matches[i].querypoint;
    const keypoint = matches[i].keypoint;

    const {x, y, scale, angle} = _mapCorrespondence({querypoint, keypoint, keycenterX, keycenterY, scaleOneOverLogK});

    // Check that the vote is within range
    if (x < minX || x >= maxX || y < minY || y >= maxY || angle <= -Math.PI || angle > Math.PI || scale < minScale || scale >= maxScale) {
      querypointValids[i] = false;
      continue;
    }

    // map properties to bins
    let fbinX = numXBins * (x - minX) / (maxX - minX);
    let fbinY = numYBins * (y - minY) / (maxY - minY);
    let fbinAngle = numAngleBins * (angle + Math.PI) / (2.0 * Math.PI);
    let fbinScale = numScaleBins * (scale - minScale) / (maxScale - minScale);

    querypointBinLocations[i] = {binX: fbinX, binY: fbinY, binAngle: fbinAngle, binScale: fbinScale};

    let binX = Math.floor(fbinX - 0.5);
    let binY = Math.floor(fbinY - 0.5);
    let binScale = Math.floor(fbinScale - 0.5);
    let binAngle = (Math.floor(fbinAngle - 0.5) + numAngleBins) % numAngleBins;

    // check can vote all 16 bins
    if (binX < 0 || binX + 1 >= numXBins || binY < 0 || binY + 1 >= numYBins || binScale < 0 || binScale +1 >= numScaleBins) {
      querypointValids[i] = false;
      continue;
    }

    for (let dx = 0; dx < 2; dx++) {
      let binX2 = binX + dx;

      for (let dy = 0; dy < 2; dy++) {
        let binY2 = binY + dy;

        for (let dangle = 0; dangle < 2; dangle++) {
          let binAngle2 = (binAngle + dangle) % numAngleBins;

          for (let dscale = 0; dscale < 2; dscale++) {
            let binScale2 = binScale + dscale;

            const binIndex = binX2 + binY2 * numXBins + binAngle2 * numXYBins + binScale2 * numXYAngleBins;

            if (votes[binIndex] === undefined) votes[binIndex] = 0;
            votes[binIndex] += 1;
          }
        }
      }
    }
    querypointValids[i] = true;
  }

  let maxVotes = 0;
  let maxVoteIndex = -1;
  Object.keys(votes).forEach((index) => {
    if (votes[index] > maxVotes) {
      maxVotes = votes[index];
      maxVoteIndex = index;
    }
  });

  if (maxVotes < 3) return [];

  // get back bins from vote index
  const binX = Math.floor(((maxVoteIndex % numXYAngleBins) % numXYBins) % numXBins);
  const binY = Math.floor((((maxVoteIndex - binX) % numXYAngleBins) % numXYBins) / numXBins);
  const binAngle = Math.floor(((maxVoteIndex - binX - (binY * numXBins)) % numXYAngleBins) / numXYBins);
  const binScale = Math.floor((maxVoteIndex - binX - (binY * numXBins) - (binAngle * numXYBins)) / numXYAngleBins);

  //console.log("hough voted: ", {binX, binY, binAngle, binScale, maxVoteIndex});

  const houghMatches = [];
  for (let i = 0; i < matches.length; i++) {
    if (!querypointValids[i]) continue;

    const queryBins = querypointBinLocations[i];
    // compute bin difference
    const distBinX = Math.abs(queryBins.binX - (binX+0.5));
    if (distBinX >= kHoughBinDelta) continue;

    const distBinY = Math.abs(queryBins.binY - (binY+0.5));
    if (distBinY >= kHoughBinDelta) continue;

    const distBinScale = Math.abs(queryBins.binScale - (binScale+0.5));
    if (distBinScale >= kHoughBinDelta) continue;

    const temp = Math.abs(queryBins.binAngle - (binAngle+0.5));
    const distBinAngle = Math.min(temp, numAngleBins - temp);
    if (distBinAngle >= kHoughBinDelta) continue;

    houghMatches.push(matches[i]);
  }
  return houghMatches;
}

const _mapCorrespondence = ({querypoint, keypoint, keycenterX, keycenterY, scaleOneOverLogK}) => {
  // map angle to (-pi, pi]
  let angle = querypoint.angle - keypoint.angle;
  if (angle <= -Math.PI) angle += 2*Math.PI;
  else if (angle > Math.PI) angle -= 2*Math.PI;

  const scale = querypoint.scale / keypoint.scale;

  // 2x2 similarity
  const cos = scale * Math.cos(angle);
  const sin = scale * Math.sin(angle);
  const S = [cos, -sin, sin, cos];

  const tp = [
    S[0] * keypoint.x + S[1] * keypoint.y,
    S[2] * keypoint.x + S[3] * keypoint.y
  ];
  const tx = querypoint.x - tp[0];
  const ty = querypoint.y - tp[1];

  return {
    x: S[0] * keycenterX + S[1] * keycenterY + tx,
    y: S[2] * keycenterX + S[3] * keycenterY + ty,
    angle: angle,
    scale: Math.log(scale) * scaleOneOverLogK
  }
}

module.exports = {
  computeHoughMatches
}


/***/ }),

/***/ "./mindar/image-target/matching/matcher.js":
/*!*************************************************!*\
  !*** ./mindar/image-target/matching/matcher.js ***!
  \*************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {match} = __webpack_require__(/*! ./matching */ "./mindar/image-target/matching/matching.js");

class Matcher {
  constructor(queryWidth, queryHeight, debugMode = false) {
    this.queryWidth = queryWidth;
    this.queryHeight = queryHeight;
    this.debugMode = debugMode;
  }

  matchDetection(keyframes, featurePoints) {
    let debugExtra = {frames: []};

    let bestResult = null;
    for (let i = 0; i < keyframes.length; i++) {
      const {H, matches, debugExtra: frameDebugExtra} = match({keyframe: keyframes[i], querypoints: featurePoints, querywidth: this.queryWidth, queryheight: this.queryHeight, debugMode: this.debugMode});
      debugExtra.frames.push(frameDebugExtra);

      if (H) {
	if (bestResult === null || bestResult.matches.length < matches.length) {
	  bestResult = {keyframeIndex: i, H, matches};
	}
      }
    }

    if (bestResult === null) {
      return {keyframeIndex: -1, debugExtra};
    }

    const screenCoords = [];
    const worldCoords = [];
    const keyframe = keyframes[bestResult.keyframeIndex];
    for (let i = 0; i < bestResult.matches.length; i++) {
      const querypoint = bestResult.matches[i].querypoint;
      const keypoint = bestResult.matches[i].keypoint;
      screenCoords.push({
        x: querypoint.x,
        y: querypoint.y,
      })
      worldCoords.push({
        x: (keypoint.x + 0.5) / keyframe.scale,
        y: (keypoint.y + 0.5) / keyframe.scale,
        z: 0,
      })
    }
    return {screenCoords, worldCoords, keyframeIndex: bestResult.keyframeIndex, debugExtra};
  }
}

module.exports = {
  Matcher
}


/***/ }),

/***/ "./mindar/image-target/matching/matching.js":
/*!**************************************************!*\
  !*** ./mindar/image-target/matching/matching.js ***!
  \**************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const TinyQueue = (__webpack_require__(/*! tinyqueue */ "./node_modules/tinyqueue/index.js")["default"]);
const {compute: hammingCompute} = __webpack_require__(/*! ./hamming-distance.js */ "./mindar/image-target/matching/hamming-distance.js");
const {computeHoughMatches} = __webpack_require__(/*! ./hough.js */ "./mindar/image-target/matching/hough.js");
const {computeHomography} = __webpack_require__(/*! ./ransacHomography.js */ "./mindar/image-target/matching/ransacHomography.js");
const {multiplyPointHomographyInhomogenous, matrixInverse33} = __webpack_require__(/*! ../utils/geometry.js */ "./mindar/image-target/utils/geometry.js");

const INLIER_THRESHOLD = 3;
//const MIN_NUM_INLIERS = 8;  //default
const MIN_NUM_INLIERS = 6;
const CLUSTER_MAX_POP = 8;
const HAMMING_THRESHOLD = 0.7;

// match list of querpoints against pre-built list of keyframes
const match = ({keyframe, querypoints, querywidth, queryheight, debugMode}) => {
  let debugExtra = {};

  const matches = [];
  for (let j = 0; j < querypoints.length; j++) {
    const querypoint = querypoints[j];
    const keypoints = querypoint.maxima? keyframe.maximaPoints: keyframe.minimaPoints;
    if (keypoints.length === 0) continue;

    const rootNode = querypoint.maxima? keyframe.maximaPointsCluster.rootNode: keyframe.minimaPointsCluster.rootNode;

    const keypointIndexes = [];
    const queue = new TinyQueue([], (a1, a2) => {return a1.d - a2.d});

    // query all potential keypoints
    _query({node: rootNode, keypoints, querypoint, queue, keypointIndexes, numPop: 0});

    let bestIndex = -1;
    let bestD1 = Number.MAX_SAFE_INTEGER;
    let bestD2 = Number.MAX_SAFE_INTEGER;

    for (let k = 0; k < keypointIndexes.length; k++) {
      const keypoint = keypoints[keypointIndexes[k]];

      const d = hammingCompute({v1: keypoint.descriptors, v2: querypoint.descriptors});
      if (d < bestD1) {
	bestD2 = bestD1;
	bestD1 = d;
	bestIndex = keypointIndexes[k];
      } else if (d < bestD2) {
	bestD2 = d;
      }
    }
    if (bestIndex !== -1 && (bestD2 === Number.MAX_SAFE_INTEGER || (1.0 * bestD1 / bestD2) < HAMMING_THRESHOLD)) {
      matches.push({querypoint, keypoint: keypoints[bestIndex]});
    }
  }

  if (debugMode) {
    debugExtra.matches = matches;
  }

  if (matches.length < MIN_NUM_INLIERS) return {debugExtra};

  const houghMatches = computeHoughMatches({
    keywidth: keyframe.width,
    keyheight: keyframe.height,
    querywidth,
    queryheight,
    matches,
  });

  if (debugMode) {
    debugExtra.houghMatches = houghMatches;
  }

  const H = computeHomography({
    srcPoints: houghMatches.map((m) => [m.keypoint.x, m.keypoint.y]),
    dstPoints: houghMatches.map((m) => [m.querypoint.x, m.querypoint.y]),
    keyframe,
  });

  if (H === null) return {debugExtra};

  const inlierMatches = _findInlierMatches({
    H,
    matches: houghMatches,
    threshold: INLIER_THRESHOLD
  });
  
  if (debugMode) {
    debugExtra.inlierMatches = inlierMatches;
  }

  if (inlierMatches.length < MIN_NUM_INLIERS) return {debugExtra}; 

  // do another loop of match using the homography
  const HInv = matrixInverse33(H, 0.00001);
  const dThreshold2 = 10 * 10;
  const matches2 = [];
  for (let j = 0; j < querypoints.length; j++) {
    const querypoint = querypoints[j];
    const mapquerypoint = multiplyPointHomographyInhomogenous([querypoint.x, querypoint.y], HInv);

    let bestIndex = -1;
    let bestD1 = Number.MAX_SAFE_INTEGER;
    let bestD2 = Number.MAX_SAFE_INTEGER;

    const keypoints = querypoint.maxima? keyframe.maximaPoints: keyframe.minimaPoints;

    for (let k = 0; k < keypoints.length; k++) {
      const keypoint = keypoints[k];

      // check distance threshold
      const d2 = (keypoint.x - mapquerypoint[0]) * (keypoint.x - mapquerypoint[0])
		+ (keypoint.y - mapquerypoint[1]) * (keypoint.y - mapquerypoint[1]);
      if (d2 > dThreshold2) continue;

      const d = hammingCompute({v1: keypoint.descriptors, v2: querypoint.descriptors});
      if (d < bestD1) {
	bestD2 = bestD1;
	bestD1 = d;
	bestIndex = k;
      } else if (d < bestD2) {
	bestD2 = d;
      }
    }

    if (bestIndex !== -1 && (bestD2 === Number.MAX_SAFE_INTEGER || (1.0 * bestD1 / bestD2) < HAMMING_THRESHOLD)) {
      matches2.push({querypoint, keypoint: keypoints[bestIndex]});
    }
  }

  if (debugMode) {
    debugExtra.matches2 = matches2;
  }

  const houghMatches2 = computeHoughMatches({
    keywidth: keyframe.width,
    keyheight: keyframe.height,
    querywidth,
    queryheight,
    matches: matches2,
  });

  if (debugMode) {
    debugExtra.houghMatches2 = houghMatches2;
  }

  const H2 = computeHomography({
    srcPoints: houghMatches2.map((m) => [m.keypoint.x, m.keypoint.y]),
    dstPoints: houghMatches2.map((m) => [m.querypoint.x, m.querypoint.y]),
    keyframe,
  });

  if (H2 === null) return {debugExtra};

  const inlierMatches2 = _findInlierMatches({
    H: H2,
    matches: houghMatches2,
    threshold: INLIER_THRESHOLD
  });

  if (debugMode) {
    debugExtra.inlierMatches2 = inlierMatches2;
  }

  return {H: H2, matches: inlierMatches2, debugExtra};
};

const _query = ({node, keypoints, querypoint, queue, keypointIndexes, numPop}) => {
  if (node.leaf) {
    for (let i = 0; i < node.pointIndexes.length; i++) {
      keypointIndexes.push(node.pointIndexes[i]);
    }
    return;
  }

  const distances = [];
  for (let i = 0; i < node.children.length; i++) {
    const childNode = node.children[i];
    const centerPointIndex = childNode.centerPointIndex;
    const d = hammingCompute({v1: keypoints[centerPointIndex].descriptors, v2: querypoint.descriptors});
    distances.push(d);
  }

  let minD = Number.MAX_SAFE_INTEGER;
  for (let i = 0; i < node.children.length; i++) {
    minD = Math.min(minD, distances[i]);
  }

  for (let i = 0; i < node.children.length; i++) {
    if (distances[i] !== minD) {
      queue.push({node: node.children[i], d: distances[i]});
    }
  }
  for (let i = 0; i < node.children.length; i++) {
    if (distances[i] === minD) {
      _query({node: node.children[i], keypoints, querypoint, queue, keypointIndexes, numPop});
    }
  }

  if (numPop < CLUSTER_MAX_POP && queue.length > 0) {
    const {node, d} = queue.pop();
    numPop += 1;
    _query({node, keypoints, querypoint, queue, keypointIndexes, numPop});
  }
};

const _findInlierMatches = (options) => {
  const {H, matches, threshold} = options;

  const threshold2 = threshold * threshold;

  const goodMatches = [];
  for (let i = 0; i < matches.length; i++) {
    const querypoint = matches[i].querypoint;
    const keypoint = matches[i].keypoint;
    const mp = multiplyPointHomographyInhomogenous([keypoint.x, keypoint.y], H);
    const d2 = (mp[0] - querypoint.x) * (mp[0] - querypoint.x) + (mp[1] - querypoint.y) * (mp[1] - querypoint.y);
    if (d2 <= threshold2) {
      goodMatches.push( matches[i] );
    }
  }
  return goodMatches;
}

module.exports = {
  match
}


/***/ }),

/***/ "./mindar/image-target/matching/ransacHomography.js":
/*!**********************************************************!*\
  !*** ./mindar/image-target/matching/ransacHomography.js ***!
  \**********************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {Matrix, inverse} = __webpack_require__(/*! ml-matrix */ "./node_modules/ml-matrix/src/index.js");
const {createRandomizer} = __webpack_require__(/*! ../utils/randomizer.js */ "./mindar/image-target/utils/randomizer.js");
const {quadrilateralConvex, matrixInverse33, smallestTriangleArea, multiplyPointHomographyInhomogenous, checkThreePointsConsistent, checkFourPointsConsistent, determinant} = __webpack_require__(/*! ../utils/geometry.js */ "./mindar/image-target/utils/geometry.js");
const {solveHomography} = __webpack_require__(/*! ../utils/homography */ "./mindar/image-target/utils/homography.js");

const CAUCHY_SCALE = 0.01;
const CHUNK_SIZE = 10;
const NUM_HYPOTHESES = 20;
const NUM_HYPOTHESES_QUICK = 10;

// Using RANSAC to estimate homography
const computeHomography = (options) => {
  const {srcPoints, dstPoints, keyframe, quickMode} = options;

  // testPoints is four corners of keyframe
  const testPoints = [
    [0, 0],
    [keyframe.width, 0],
    [keyframe.width, keyframe.height],
    [0, keyframe.height]
  ]

  const sampleSize = 4; // use four points to compute homography
  if (srcPoints.length < sampleSize) return null;

  const scale = CAUCHY_SCALE;
  const oneOverScale2 = 1.0 / (scale * scale);
  const chuckSize = Math.min(CHUNK_SIZE, srcPoints.length);

  const randomizer = createRandomizer();

  const perm = [];
  for (let i = 0; i < srcPoints.length; i++) {
    perm[i] = i;
  }

  randomizer.arrayShuffle({arr: perm, sampleSize: perm.length});

  const numHypothesis = quickMode? NUM_HYPOTHESES_QUICK: NUM_HYPOTHESES;
  const maxTrials = numHypothesis * 2;

  // build numerous hypotheses by randoming draw four points
  // TODO: optimize: if number of points is less than certain number, can brute force all combinations
  let trial = 0;
  const Hs = [];
  while (trial < maxTrials && Hs.length < numHypothesis) {
    trial +=1;

    randomizer.arrayShuffle({arr: perm, sampleSize: sampleSize});

    // their relative positions match each other
    if (!checkFourPointsConsistent(
      srcPoints[perm[0]], srcPoints[perm[1]], srcPoints[perm[2]], srcPoints[perm[3]],
      dstPoints[perm[0]], dstPoints[perm[1]], dstPoints[perm[2]], dstPoints[perm[3]])) {
      continue;
    }

    const H = solveHomography(
      [srcPoints[perm[0]], srcPoints[perm[1]], srcPoints[perm[2]], srcPoints[perm[3]]],
      [dstPoints[perm[0]], dstPoints[perm[1]], dstPoints[perm[2]], dstPoints[perm[3]]],
    );
    if (H === null) continue;

    if(!_checkHomographyPointsGeometricallyConsistent({H, testPoints})) {
      continue;
    }

    Hs.push(H);
  }

  if (Hs.length === 0) return null;

  // pick the best hypothesis
  const hypotheses = [];
  for (let i = 0; i < Hs.length; i++) {
    hypotheses.push({
      H: Hs[i],
      cost: 0
    })
  }

  let curChuckSize = chuckSize;
  for (let i = 0; i < srcPoints.length && hypotheses.length > 2; i += curChuckSize) {
    curChuckSize = Math.min(chuckSize, srcPoints.length - i);
    let chuckEnd = i + curChuckSize;

    for (let j = 0; j < hypotheses.length; j++) {
      for (let k = i; k < chuckEnd; k++) {
        const cost = _cauchyProjectiveReprojectionCost({H: hypotheses[j].H, srcPoint: srcPoints[k], dstPoint: dstPoints[k], oneOverScale2});
        hypotheses[j].cost += cost;
      }
    }

    hypotheses.sort((h1, h2) => {return h1.cost - h2.cost});
    hypotheses.splice(-Math.floor((hypotheses.length+1)/2)); // keep the best half
  }

  let finalH = null;
  for (let i = 0; i < hypotheses.length; i++) {
    const H = _normalizeHomography({inH: hypotheses[i].H});
    if (_checkHeuristics({H: H, testPoints, keyframe})) {
      finalH = H;
      break;
    }
  }
  return finalH;
}

const _checkHeuristics = ({H, testPoints, keyframe}) => {
  const HInv = matrixInverse33(H, 0.00001);
  if (HInv === null) return false;

  const mp = []
  for (let i = 0; i < testPoints.length; i++) { // 4 test points, corner of keyframe
    mp.push(multiplyPointHomographyInhomogenous(testPoints[i], HInv));
  }
  const smallArea = smallestTriangleArea(mp[0], mp[1], mp[2], mp[3]);

  if (smallArea < keyframe.width * keyframe.height * 0.0001) return false;

  if (!quadrilateralConvex(mp[0], mp[1], mp[2], mp[3])) return false;

  return true;
}

const _normalizeHomography = ({inH}) => {
  const oneOver = 1.0 / inH[8];

  const H = [];
  for (let i = 0; i < 8; i++) {
    H[i] = inH[i] * oneOver;
  }
  H[8] = 1.0;
  return H;
}

const _cauchyProjectiveReprojectionCost = ({H, srcPoint, dstPoint, oneOverScale2}) => {
  const x = multiplyPointHomographyInhomogenous(srcPoint, H);
  const f =[
    x[0] - dstPoint[0],
    x[1] - dstPoint[1]
  ];
  return Math.log(1 + (f[0]*f[0]+f[1]*f[1]) * oneOverScale2);
}

const _checkHomographyPointsGeometricallyConsistent = ({H, testPoints}) => {
  const mappedPoints = [];
  for (let i = 0; i < testPoints.length; i++) {
    mappedPoints[i] = multiplyPointHomographyInhomogenous(testPoints[i], H);
  }
  for (let i = 0; i < testPoints.length; i++) {
    const i1 = i;
    const i2 = (i+1) % testPoints.length;
    const i3 = (i+2) % testPoints.length;
    if (!checkThreePointsConsistent(
      testPoints[i1], testPoints[i2], testPoints[i3],
      mappedPoints[i1], mappedPoints[i2], mappedPoints[i3])) return false;
  }
  return true;
}

module.exports = {
  computeHomography,
}


/***/ }),

/***/ "./mindar/image-target/utils/geometry.js":
/*!***********************************************!*\
  !*** ./mindar/image-target/utils/geometry.js ***!
  \***********************************************/
/***/ ((module) => {

// check which side point C on the line from A to B
const linePointSide = (A, B, C) => {
  return ((B[0]-A[0])*(C[1]-A[1])-(B[1]-A[1])*(C[0]-A[0]));
}

// srcPoints, dstPoints: array of four elements [x, y]
const checkFourPointsConsistent = (x1, x2, x3, x4, x1p, x2p, x3p, x4p) => {
  if ((linePointSide(x1, x2, x3) > 0) !== (linePointSide(x1p, x2p, x3p) > 0)) return false;
  if ((linePointSide(x2, x3, x4) > 0) !== (linePointSide(x2p, x3p, x4p) > 0)) return false;
  if ((linePointSide(x3, x4, x1) > 0) !== (linePointSide(x3p, x4p, x1p) > 0)) return false;
  if ((linePointSide(x4, x1, x2) > 0) !== (linePointSide(x4p, x1p, x2p) > 0)) return false;
  return true;
}

const checkThreePointsConsistent = (x1, x2, x3, x1p, x2p, x3p) => {
  if ((linePointSide(x1, x2, x3) > 0) !== (linePointSide(x1p, x2p, x3p) > 0)) return false;
  return true;
}

const determinant = (A) => {
  const C1 =  A[4] * A[8] - A[5] * A[7];
  const C2 =  A[3] * A[8] - A[5] * A[6];
  const C3 =  A[3] * A[7] - A[4] * A[6];
  return A[0] * C1 - A[1] * C2 + A[2] * C3;
}

const matrixInverse33 = (A, threshold) => {
  const det = determinant(A);
  if (Math.abs(det) <= threshold) return null;
  const oneOver = 1.0 / det;

  const B = [
    (A[4] * A[8] - A[5] * A[7]) * oneOver,
    (A[2] * A[7] - A[1] * A[8]) * oneOver,
    (A[1] * A[5] - A[2] * A[4]) * oneOver,
    (A[5] * A[6] - A[3] * A[8]) * oneOver,
    (A[0] * A[8] - A[2] * A[6]) * oneOver,
    (A[2] * A[3] - A[0] * A[5]) * oneOver,
    (A[3] * A[7] - A[4] * A[6]) * oneOver,
    (A[1] * A[6] - A[0] * A[7]) * oneOver,
    (A[0] * A[4] - A[1] * A[3]) * oneOver,
  ];
  return B;
}

const matrixMul33 = (A, B) => {
  const C = [];
  C[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
  C[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
  C[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];
  C[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
  C[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
  C[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];
  C[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
  C[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
  C[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];
  return C;
}

const multiplyPointHomographyInhomogenous = (x, H) => {
  const w = H[6]*x[0] + H[7]*x[1] + H[8];
  const xp = [];
  xp[0] = (H[0]*x[0] + H[1]*x[1] + H[2])/w;
  xp[1] = (H[3]*x[0] + H[4]*x[1] + H[5])/w;
  return xp;
}

const smallestTriangleArea = (x1, x2, x3, x4) => {
  const v12 = _vector(x2, x1);
  const v13 = _vector(x3, x1);
  const v14 = _vector(x4, x1);
  const v32 = _vector(x2, x3);
  const v34 = _vector(x4, x3);
  const a1 = _areaOfTriangle(v12, v13);
  const a2 = _areaOfTriangle(v13, v14);
  const a3 = _areaOfTriangle(v12, v14);
  const a4 = _areaOfTriangle(v32, v34);
  return Math.min(Math.min(Math.min(a1, a2), a3), a4);
}

// check if four points form a convex quadrilaternal.
// all four combinations should have same sign
const quadrilateralConvex = (x1, x2, x3, x4) => {
  const first = linePointSide(x1, x2, x3) <= 0;
  if ( (linePointSide(x2, x3, x4) <= 0) !== first) return false;
  if ( (linePointSide(x3, x4, x1) <= 0) !== first) return false;
  if ( (linePointSide(x4, x1, x2) <= 0) !== first) return false;

  //if (linePointSide(x1, x2, x3) <= 0) return false;
  //if (linePointSide(x2, x3, x4) <= 0) return false;
  //if (linePointSide(x3, x4, x1) <= 0) return false;
  //if (linePointSide(x4, x1, x2) <= 0) return false;
  return true;
}

const _vector = (a, b) => {
  return [
    a[0] - b[0],
    a[1] - b[1]
  ]
}

const _areaOfTriangle = (u, v) => {
  const a = u[0]*v[1] - u[1]*v[0];
  return Math.abs(a) * 0.5;
}

module.exports = {
  matrixInverse33,
  matrixMul33,
  quadrilateralConvex,
  smallestTriangleArea,
  multiplyPointHomographyInhomogenous,
  checkThreePointsConsistent,
  checkFourPointsConsistent,
  determinant
}



/***/ }),

/***/ "./mindar/image-target/utils/homography.js":
/*!*************************************************!*\
  !*** ./mindar/image-target/utils/homography.js ***!
  \*************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {Matrix, inverse} = __webpack_require__(/*! ml-matrix */ "./node_modules/ml-matrix/src/index.js");

const solveHomography = (srcPoints, dstPoints) => {
  const {normPoints: normSrcPoints, param: srcParam} = _normalizePoints(srcPoints);
  const {normPoints: normDstPoints, param: dstParam} = _normalizePoints(dstPoints);

  const num = normDstPoints.length;
  const AData = [];
  const BData = [];
  for (let j = 0; j < num; j++) {
    const row1 = [
      normSrcPoints[j][0],
      normSrcPoints[j][1],
      1,
      0,
      0,
      0,
      -(normSrcPoints[j][0] * normDstPoints[j][0]),
      -(normSrcPoints[j][1] * normDstPoints[j][0]),
    ];
    const row2 = [
      0,
      0,
      0,
      normSrcPoints[j][0],
      normSrcPoints[j][1],
      1,
      -(normSrcPoints[j][0] * normDstPoints[j][1]),
      -(normSrcPoints[j][1] * normDstPoints[j][1]),
    ];
    AData.push(row1);
    AData.push(row2);

    BData.push([normDstPoints[j][0]]);
    BData.push([normDstPoints[j][1]]);
  }

  try {
    const A = new Matrix(AData);
    const B = new Matrix(BData);
    const AT = A.transpose();
    const ATA = AT.mmul(A);
    const ATB = AT.mmul(B);
    const ATAInv = inverse(ATA);
    const C = ATAInv.mmul(ATB).to1DArray();
    const H = _denormalizeHomography(C, srcParam, dstParam);
    return H;
  } catch (e) {
    return null;
  }
}

// centroid at origin and avg distance from origin is sqrt(2)
const _normalizePoints = (coords) => {
  //return {normalizedCoords: coords, param: {meanX: 0, meanY: 0, s: 1}}; // skip normalization

  let sumX = 0;
  let sumY = 0;
  for (let i = 0; i < coords.length; i++) {
    sumX += coords[i][0];
    sumY += coords[i][1];
  }
  let meanX = sumX / coords.length;
  let meanY = sumY / coords.length;

  let sumDiff = 0;
  for (let i = 0; i < coords.length; i++) {
    const diffX = coords[i][0] - meanX;
    const diffY = coords[i][1] - meanY;
    sumDiff += Math.sqrt(diffX * diffX + diffY * diffY);
  }
  let s = Math.sqrt(2) * coords.length / sumDiff;

  const normPoints = [];
  for (let i = 0; i < coords.length; i++) {
    normPoints.push([
      (coords[i][0] - meanX) * s,
      (coords[i][1] - meanY) * s,
    ]);
  }
  return {normPoints, param: {meanX, meanY, s}};
}

// Denormalize homography
// where T is the normalization matrix, i.e.
//
//     [1  0  -meanX]
// T = [0  1  -meanY]
//     [0  0     1/s]
//
//          [1  0  s*meanX]
// inv(T) = [0  1  s*meanY]
// 	    [0  0        s]
//
// H = inv(Tdst) * Hn * Tsrc
//
// @param {
//   nH: normH,
//   srcParam: param of src transform,
//   dstParam: param of dst transform
// }
const _denormalizeHomography = (nH, srcParam, dstParam) => {
  /*
  Matrix version
  const normH = new Matrix([
    [nH[0], nH[1], nH[2]],
    [nH[3], nH[4], nH[5]],
    [nH[6], nH[7], 1],
  ]);
  const Tsrc = new Matrix([
    [1, 0, -srcParam.meanX],
    [0, 1, -srcParam.meanY],
    [0, 0,    1/srcParam.s],
  ]);

  const invTdst = new Matrix([
    [1, 0, dstParam.s * dstParam.meanX],
    [0, 1, dstParam.s * dstParam.meanY],
    [0, 0, dstParam.s],
  ]);
  const H = invTdst.mmul(normH).mmul(Tsrc);
  */

  // plain implementation of the above using Matrix
  const sMeanX = dstParam.s * dstParam.meanX;
  const sMeanY = dstParam.s * dstParam.meanY;

  const H = [
      nH[0] + sMeanX * nH[6], 
      nH[1] + sMeanX * nH[7],
      (nH[0] + sMeanX * nH[6]) * -srcParam.meanX + (nH[1] + sMeanX * nH[7]) * -srcParam.meanY + (nH[2] + sMeanX) / srcParam.s,
      nH[3] + sMeanY * nH[6], 
      nH[4] + sMeanY * nH[7],
      (nH[3] + sMeanY * nH[6]) * -srcParam.meanX + (nH[4] + sMeanY * nH[7]) * -srcParam.meanY + (nH[5] + sMeanY) / srcParam.s,
      dstParam.s * nH[6],
      dstParam.s * nH[7],
      dstParam.s * nH[6] * -srcParam.meanX + dstParam.s * nH[7] * -srcParam.meanY + dstParam.s / srcParam.s,
  ];

  // make H[8] === 1;
  for (let i = 0; i < 9; i++) {
    H[i] = H[i] / H[8];
  }
  return H;
}

module.exports = {
  solveHomography
}


/***/ }),

/***/ "./mindar/image-target/utils/randomizer.js":
/*!*************************************************!*\
  !*** ./mindar/image-target/utils/randomizer.js ***!
  \*************************************************/
/***/ ((module) => {

const mRandSeed = 1234;

const createRandomizer = () => {
  const randomizer = {
    seed: mRandSeed,

    arrayShuffle(options) {
      const {arr, sampleSize} = options;
      for (let i = 0; i < sampleSize; i++) {

        this.seed = (214013 * this.seed + 2531011) % (1 << 31);
        let k = (this.seed >> 16) & 0x7fff;
        k = k % arr.length;

        let tmp = arr[i];
        arr[i] = arr[k];
        arr[k] = tmp;
      }
    },

    nextInt(maxValue) {
      this.seed = (214013 * this.seed + 2531011) % (1 << 31);
      let k = (this.seed >> 16) & 0x7fff;
      k = k % maxValue;
      return k;
    }
  }
  return randomizer;
}

module.exports = {
  createRandomizer
}


/***/ }),

/***/ "./node_modules/is-any-array/lib-esm/index.js":
/*!****************************************************!*\
  !*** ./node_modules/is-any-array/lib-esm/index.js ***!
  \****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "isAnyArray": () => (/* binding */ isAnyArray)
/* harmony export */ });
const toString = Object.prototype.toString;
/**
 * Checks if an object is an instance of an Array (array or typed array).
 *
 * @param {any} value - Object to check.
 * @returns {boolean} True if the object is an array.
 */
function isAnyArray(value) {
    return toString.call(value).endsWith('Array]');
}
//# sourceMappingURL=index.js.map

/***/ }),

/***/ "./node_modules/ml-array-max/lib-es6/index.js":
/*!****************************************************!*\
  !*** ./node_modules/ml-array-max/lib-es6/index.js ***!
  \****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ max)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");


function max(input) {
  var options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};

  if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(input)) {
    throw new TypeError('input must be an array');
  }

  if (input.length === 0) {
    throw new TypeError('input must not be empty');
  }

  var _options$fromIndex = options.fromIndex,
      fromIndex = _options$fromIndex === void 0 ? 0 : _options$fromIndex,
      _options$toIndex = options.toIndex,
      toIndex = _options$toIndex === void 0 ? input.length : _options$toIndex;

  if (fromIndex < 0 || fromIndex >= input.length || !Number.isInteger(fromIndex)) {
    throw new Error('fromIndex must be a positive integer smaller than length');
  }

  if (toIndex <= fromIndex || toIndex > input.length || !Number.isInteger(toIndex)) {
    throw new Error('toIndex must be an integer greater than fromIndex and at most equal to length');
  }

  var maxValue = input[fromIndex];

  for (var i = fromIndex + 1; i < toIndex; i++) {
    if (input[i] > maxValue) maxValue = input[i];
  }

  return maxValue;
}




/***/ }),

/***/ "./node_modules/ml-array-min/lib-es6/index.js":
/*!****************************************************!*\
  !*** ./node_modules/ml-array-min/lib-es6/index.js ***!
  \****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ min)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");


function min(input) {
  var options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};

  if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(input)) {
    throw new TypeError('input must be an array');
  }

  if (input.length === 0) {
    throw new TypeError('input must not be empty');
  }

  var _options$fromIndex = options.fromIndex,
      fromIndex = _options$fromIndex === void 0 ? 0 : _options$fromIndex,
      _options$toIndex = options.toIndex,
      toIndex = _options$toIndex === void 0 ? input.length : _options$toIndex;

  if (fromIndex < 0 || fromIndex >= input.length || !Number.isInteger(fromIndex)) {
    throw new Error('fromIndex must be a positive integer smaller than length');
  }

  if (toIndex <= fromIndex || toIndex > input.length || !Number.isInteger(toIndex)) {
    throw new Error('toIndex must be an integer greater than fromIndex and at most equal to length');
  }

  var minValue = input[fromIndex];

  for (var i = fromIndex + 1; i < toIndex; i++) {
    if (input[i] < minValue) minValue = input[i];
  }

  return minValue;
}




/***/ }),

/***/ "./node_modules/ml-array-rescale/lib-es6/index.js":
/*!********************************************************!*\
  !*** ./node_modules/ml-array-rescale/lib-es6/index.js ***!
  \********************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ rescale)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");
/* harmony import */ var ml_array_max__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ml-array-max */ "./node_modules/ml-array-max/lib-es6/index.js");
/* harmony import */ var ml_array_min__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ml-array-min */ "./node_modules/ml-array-min/lib-es6/index.js");




function rescale(input) {
  var options = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : {};

  if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(input)) {
    throw new TypeError('input must be an array');
  } else if (input.length === 0) {
    throw new TypeError('input must not be empty');
  }

  var output;

  if (options.output !== undefined) {
    if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(options.output)) {
      throw new TypeError('output option must be an array if specified');
    }

    output = options.output;
  } else {
    output = new Array(input.length);
  }

  var currentMin = (0,ml_array_min__WEBPACK_IMPORTED_MODULE_2__["default"])(input);
  var currentMax = (0,ml_array_max__WEBPACK_IMPORTED_MODULE_1__["default"])(input);

  if (currentMin === currentMax) {
    throw new RangeError('minimum and maximum input values are equal. Cannot rescale a constant array');
  }

  var _options$min = options.min,
      minValue = _options$min === void 0 ? options.autoMinMax ? currentMin : 0 : _options$min,
      _options$max = options.max,
      maxValue = _options$max === void 0 ? options.autoMinMax ? currentMax : 1 : _options$max;

  if (minValue >= maxValue) {
    throw new RangeError('min option must be smaller than max option');
  }

  var factor = (maxValue - minValue) / (currentMax - currentMin);

  for (var i = 0; i < input.length; i++) {
    output[i] = (input[i] - currentMin) * factor + minValue;
  }

  return output;
}




/***/ }),

/***/ "./node_modules/ml-matrix/src/correlation.js":
/*!***************************************************!*\
  !*** ./node_modules/ml-matrix/src/correlation.js ***!
  \***************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "correlation": () => (/* binding */ correlation)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./matrix */ "./node_modules/ml-matrix/src/matrix.js");




function correlation(xMatrix, yMatrix = xMatrix, options = {}) {
  xMatrix = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](xMatrix);
  let yIsSame = false;
  if (
    typeof yMatrix === 'object' &&
    !_matrix__WEBPACK_IMPORTED_MODULE_1__["default"].isMatrix(yMatrix) &&
    !(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(yMatrix)
  ) {
    options = yMatrix;
    yMatrix = xMatrix;
    yIsSame = true;
  } else {
    yMatrix = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](yMatrix);
  }
  if (xMatrix.rows !== yMatrix.rows) {
    throw new TypeError('Both matrices must have the same number of rows');
  }

  const { center = true, scale = true } = options;
  if (center) {
    xMatrix.center('column');
    if (!yIsSame) {
      yMatrix.center('column');
    }
  }
  if (scale) {
    xMatrix.scale('column');
    if (!yIsSame) {
      yMatrix.scale('column');
    }
  }

  const sdx = xMatrix.standardDeviation('column', { unbiased: true });
  const sdy = yIsSame
    ? sdx
    : yMatrix.standardDeviation('column', { unbiased: true });

  const corr = xMatrix.transpose().mmul(yMatrix);
  for (let i = 0; i < corr.rows; i++) {
    for (let j = 0; j < corr.columns; j++) {
      corr.set(
        i,
        j,
        corr.get(i, j) * (1 / (sdx[i] * sdy[j])) * (1 / (xMatrix.rows - 1)),
      );
    }
  }
  return corr;
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/covariance.js":
/*!**************************************************!*\
  !*** ./node_modules/ml-matrix/src/covariance.js ***!
  \**************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "covariance": () => (/* binding */ covariance)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./matrix */ "./node_modules/ml-matrix/src/matrix.js");




function covariance(xMatrix, yMatrix = xMatrix, options = {}) {
  xMatrix = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](xMatrix);
  let yIsSame = false;
  if (
    typeof yMatrix === 'object' &&
    !_matrix__WEBPACK_IMPORTED_MODULE_1__["default"].isMatrix(yMatrix) &&
    !(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(yMatrix)
  ) {
    options = yMatrix;
    yMatrix = xMatrix;
    yIsSame = true;
  } else {
    yMatrix = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](yMatrix);
  }
  if (xMatrix.rows !== yMatrix.rows) {
    throw new TypeError('Both matrices must have the same number of rows');
  }
  const { center = true } = options;
  if (center) {
    xMatrix = xMatrix.center('column');
    if (!yIsSame) {
      yMatrix = yMatrix.center('column');
    }
  }
  const cov = xMatrix.transpose().mmul(yMatrix);
  for (let i = 0; i < cov.rows; i++) {
    for (let j = 0; j < cov.columns; j++) {
      cov.set(i, j, cov.get(i, j) * (1 / (xMatrix.rows - 1)));
    }
  }
  return cov;
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/dc/cholesky.js":
/*!***************************************************!*\
  !*** ./node_modules/ml-matrix/src/dc/cholesky.js ***!
  \***************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ CholeskyDecomposition)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");



class CholeskyDecomposition {
  constructor(value) {
    value = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(value);
    if (!value.isSymmetric()) {
      throw new Error('Matrix is not symmetric');
    }

    let a = value;
    let dimension = a.rows;
    let l = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](dimension, dimension);
    let positiveDefinite = true;
    let i, j, k;

    for (j = 0; j < dimension; j++) {
      let d = 0;
      for (k = 0; k < j; k++) {
        let s = 0;
        for (i = 0; i < k; i++) {
          s += l.get(k, i) * l.get(j, i);
        }
        s = (a.get(j, k) - s) / l.get(k, k);
        l.set(j, k, s);
        d = d + s * s;
      }

      d = a.get(j, j) - d;

      positiveDefinite &= d > 0;
      l.set(j, j, Math.sqrt(Math.max(d, 0)));
      for (k = j + 1; k < dimension; k++) {
        l.set(j, k, 0);
      }
    }

    this.L = l;
    this.positiveDefinite = Boolean(positiveDefinite);
  }

  isPositiveDefinite() {
    return this.positiveDefinite;
  }

  solve(value) {
    value = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(value);

    let l = this.L;
    let dimension = l.rows;

    if (value.rows !== dimension) {
      throw new Error('Matrix dimensions do not match');
    }
    if (this.isPositiveDefinite() === false) {
      throw new Error('Matrix is not positive definite');
    }

    let count = value.columns;
    let B = value.clone();
    let i, j, k;

    for (k = 0; k < dimension; k++) {
      for (j = 0; j < count; j++) {
        for (i = 0; i < k; i++) {
          B.set(k, j, B.get(k, j) - B.get(i, j) * l.get(k, i));
        }
        B.set(k, j, B.get(k, j) / l.get(k, k));
      }
    }

    for (k = dimension - 1; k >= 0; k--) {
      for (j = 0; j < count; j++) {
        for (i = k + 1; i < dimension; i++) {
          B.set(k, j, B.get(k, j) - B.get(i, j) * l.get(i, k));
        }
        B.set(k, j, B.get(k, j) / l.get(k, k));
      }
    }

    return B;
  }

  get lowerTriangularMatrix() {
    return this.L;
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/dc/evd.js":
/*!**********************************************!*\
  !*** ./node_modules/ml-matrix/src/dc/evd.js ***!
  \**********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ EigenvalueDecomposition)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./util */ "./node_modules/ml-matrix/src/dc/util.js");





class EigenvalueDecomposition {
  constructor(matrix, options = {}) {
    const { assumeSymmetric = false } = options;

    matrix = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(matrix);
    if (!matrix.isSquare()) {
      throw new Error('Matrix is not a square matrix');
    }

    if (matrix.isEmpty()) {
      throw new Error('Matrix must be non-empty');
    }

    let n = matrix.columns;
    let V = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](n, n);
    let d = new Float64Array(n);
    let e = new Float64Array(n);
    let value = matrix;
    let i, j;

    let isSymmetric = false;
    if (assumeSymmetric) {
      isSymmetric = true;
    } else {
      isSymmetric = matrix.isSymmetric();
    }

    if (isSymmetric) {
      for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
          V.set(i, j, value.get(i, j));
        }
      }
      tred2(n, e, d, V);
      tql2(n, e, d, V);
    } else {
      let H = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](n, n);
      let ort = new Float64Array(n);
      for (j = 0; j < n; j++) {
        for (i = 0; i < n; i++) {
          H.set(i, j, value.get(i, j));
        }
      }
      orthes(n, H, ort, V);
      hqr2(n, e, d, V, H);
    }

    this.n = n;
    this.e = e;
    this.d = d;
    this.V = V;
  }

  get realEigenvalues() {
    return Array.from(this.d);
  }

  get imaginaryEigenvalues() {
    return Array.from(this.e);
  }

  get eigenvectorMatrix() {
    return this.V;
  }

  get diagonalMatrix() {
    let n = this.n;
    let e = this.e;
    let d = this.d;
    let X = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](n, n);
    let i, j;
    for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++) {
        X.set(i, j, 0);
      }
      X.set(i, i, d[i]);
      if (e[i] > 0) {
        X.set(i, i + 1, e[i]);
      } else if (e[i] < 0) {
        X.set(i, i - 1, e[i]);
      }
    }
    return X;
  }
}

function tred2(n, e, d, V) {
  let f, g, h, i, j, k, hh, scale;

  for (j = 0; j < n; j++) {
    d[j] = V.get(n - 1, j);
  }

  for (i = n - 1; i > 0; i--) {
    scale = 0;
    h = 0;
    for (k = 0; k < i; k++) {
      scale = scale + Math.abs(d[k]);
    }

    if (scale === 0) {
      e[i] = d[i - 1];
      for (j = 0; j < i; j++) {
        d[j] = V.get(i - 1, j);
        V.set(i, j, 0);
        V.set(j, i, 0);
      }
    } else {
      for (k = 0; k < i; k++) {
        d[k] /= scale;
        h += d[k] * d[k];
      }

      f = d[i - 1];
      g = Math.sqrt(h);
      if (f > 0) {
        g = -g;
      }

      e[i] = scale * g;
      h = h - f * g;
      d[i - 1] = f - g;
      for (j = 0; j < i; j++) {
        e[j] = 0;
      }

      for (j = 0; j < i; j++) {
        f = d[j];
        V.set(j, i, f);
        g = e[j] + V.get(j, j) * f;
        for (k = j + 1; k <= i - 1; k++) {
          g += V.get(k, j) * d[k];
          e[k] += V.get(k, j) * f;
        }
        e[j] = g;
      }

      f = 0;
      for (j = 0; j < i; j++) {
        e[j] /= h;
        f += e[j] * d[j];
      }

      hh = f / (h + h);
      for (j = 0; j < i; j++) {
        e[j] -= hh * d[j];
      }

      for (j = 0; j < i; j++) {
        f = d[j];
        g = e[j];
        for (k = j; k <= i - 1; k++) {
          V.set(k, j, V.get(k, j) - (f * e[k] + g * d[k]));
        }
        d[j] = V.get(i - 1, j);
        V.set(i, j, 0);
      }
    }
    d[i] = h;
  }

  for (i = 0; i < n - 1; i++) {
    V.set(n - 1, i, V.get(i, i));
    V.set(i, i, 1);
    h = d[i + 1];
    if (h !== 0) {
      for (k = 0; k <= i; k++) {
        d[k] = V.get(k, i + 1) / h;
      }

      for (j = 0; j <= i; j++) {
        g = 0;
        for (k = 0; k <= i; k++) {
          g += V.get(k, i + 1) * V.get(k, j);
        }
        for (k = 0; k <= i; k++) {
          V.set(k, j, V.get(k, j) - g * d[k]);
        }
      }
    }

    for (k = 0; k <= i; k++) {
      V.set(k, i + 1, 0);
    }
  }

  for (j = 0; j < n; j++) {
    d[j] = V.get(n - 1, j);
    V.set(n - 1, j, 0);
  }

  V.set(n - 1, n - 1, 1);
  e[0] = 0;
}

function tql2(n, e, d, V) {
  let g, h, i, j, k, l, m, p, r, dl1, c, c2, c3, el1, s, s2, iter;

  for (i = 1; i < n; i++) {
    e[i - 1] = e[i];
  }

  e[n - 1] = 0;

  let f = 0;
  let tst1 = 0;
  let eps = Number.EPSILON;

  for (l = 0; l < n; l++) {
    tst1 = Math.max(tst1, Math.abs(d[l]) + Math.abs(e[l]));
    m = l;
    while (m < n) {
      if (Math.abs(e[m]) <= eps * tst1) {
        break;
      }
      m++;
    }

    if (m > l) {
      iter = 0;
      do {
        iter = iter + 1;

        g = d[l];
        p = (d[l + 1] - g) / (2 * e[l]);
        r = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(p, 1);
        if (p < 0) {
          r = -r;
        }

        d[l] = e[l] / (p + r);
        d[l + 1] = e[l] * (p + r);
        dl1 = d[l + 1];
        h = g - d[l];
        for (i = l + 2; i < n; i++) {
          d[i] -= h;
        }

        f = f + h;

        p = d[m];
        c = 1;
        c2 = c;
        c3 = c;
        el1 = e[l + 1];
        s = 0;
        s2 = 0;
        for (i = m - 1; i >= l; i--) {
          c3 = c2;
          c2 = c;
          s2 = s;
          g = c * e[i];
          h = c * p;
          r = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(p, e[i]);
          e[i + 1] = s * r;
          s = e[i] / r;
          c = p / r;
          p = c * d[i] - s * g;
          d[i + 1] = h + s * (c * g + s * d[i]);

          for (k = 0; k < n; k++) {
            h = V.get(k, i + 1);
            V.set(k, i + 1, s * V.get(k, i) + c * h);
            V.set(k, i, c * V.get(k, i) - s * h);
          }
        }

        p = (-s * s2 * c3 * el1 * e[l]) / dl1;
        e[l] = s * p;
        d[l] = c * p;
      } while (Math.abs(e[l]) > eps * tst1);
    }
    d[l] = d[l] + f;
    e[l] = 0;
  }

  for (i = 0; i < n - 1; i++) {
    k = i;
    p = d[i];
    for (j = i + 1; j < n; j++) {
      if (d[j] < p) {
        k = j;
        p = d[j];
      }
    }

    if (k !== i) {
      d[k] = d[i];
      d[i] = p;
      for (j = 0; j < n; j++) {
        p = V.get(j, i);
        V.set(j, i, V.get(j, k));
        V.set(j, k, p);
      }
    }
  }
}

function orthes(n, H, ort, V) {
  let low = 0;
  let high = n - 1;
  let f, g, h, i, j, m;
  let scale;

  for (m = low + 1; m <= high - 1; m++) {
    scale = 0;
    for (i = m; i <= high; i++) {
      scale = scale + Math.abs(H.get(i, m - 1));
    }

    if (scale !== 0) {
      h = 0;
      for (i = high; i >= m; i--) {
        ort[i] = H.get(i, m - 1) / scale;
        h += ort[i] * ort[i];
      }

      g = Math.sqrt(h);
      if (ort[m] > 0) {
        g = -g;
      }

      h = h - ort[m] * g;
      ort[m] = ort[m] - g;

      for (j = m; j < n; j++) {
        f = 0;
        for (i = high; i >= m; i--) {
          f += ort[i] * H.get(i, j);
        }

        f = f / h;
        for (i = m; i <= high; i++) {
          H.set(i, j, H.get(i, j) - f * ort[i]);
        }
      }

      for (i = 0; i <= high; i++) {
        f = 0;
        for (j = high; j >= m; j--) {
          f += ort[j] * H.get(i, j);
        }

        f = f / h;
        for (j = m; j <= high; j++) {
          H.set(i, j, H.get(i, j) - f * ort[j]);
        }
      }

      ort[m] = scale * ort[m];
      H.set(m, m - 1, scale * g);
    }
  }

  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      V.set(i, j, i === j ? 1 : 0);
    }
  }

  for (m = high - 1; m >= low + 1; m--) {
    if (H.get(m, m - 1) !== 0) {
      for (i = m + 1; i <= high; i++) {
        ort[i] = H.get(i, m - 1);
      }

      for (j = m; j <= high; j++) {
        g = 0;
        for (i = m; i <= high; i++) {
          g += ort[i] * V.get(i, j);
        }

        g = g / ort[m] / H.get(m, m - 1);
        for (i = m; i <= high; i++) {
          V.set(i, j, V.get(i, j) + g * ort[i]);
        }
      }
    }
  }
}

function hqr2(nn, e, d, V, H) {
  let n = nn - 1;
  let low = 0;
  let high = nn - 1;
  let eps = Number.EPSILON;
  let exshift = 0;
  let norm = 0;
  let p = 0;
  let q = 0;
  let r = 0;
  let s = 0;
  let z = 0;
  let iter = 0;
  let i, j, k, l, m, t, w, x, y;
  let ra, sa, vr, vi;
  let notlast, cdivres;

  for (i = 0; i < nn; i++) {
    if (i < low || i > high) {
      d[i] = H.get(i, i);
      e[i] = 0;
    }

    for (j = Math.max(i - 1, 0); j < nn; j++) {
      norm = norm + Math.abs(H.get(i, j));
    }
  }

  while (n >= low) {
    l = n;
    while (l > low) {
      s = Math.abs(H.get(l - 1, l - 1)) + Math.abs(H.get(l, l));
      if (s === 0) {
        s = norm;
      }
      if (Math.abs(H.get(l, l - 1)) < eps * s) {
        break;
      }
      l--;
    }

    if (l === n) {
      H.set(n, n, H.get(n, n) + exshift);
      d[n] = H.get(n, n);
      e[n] = 0;
      n--;
      iter = 0;
    } else if (l === n - 1) {
      w = H.get(n, n - 1) * H.get(n - 1, n);
      p = (H.get(n - 1, n - 1) - H.get(n, n)) / 2;
      q = p * p + w;
      z = Math.sqrt(Math.abs(q));
      H.set(n, n, H.get(n, n) + exshift);
      H.set(n - 1, n - 1, H.get(n - 1, n - 1) + exshift);
      x = H.get(n, n);

      if (q >= 0) {
        z = p >= 0 ? p + z : p - z;
        d[n - 1] = x + z;
        d[n] = d[n - 1];
        if (z !== 0) {
          d[n] = x - w / z;
        }
        e[n - 1] = 0;
        e[n] = 0;
        x = H.get(n, n - 1);
        s = Math.abs(x) + Math.abs(z);
        p = x / s;
        q = z / s;
        r = Math.sqrt(p * p + q * q);
        p = p / r;
        q = q / r;

        for (j = n - 1; j < nn; j++) {
          z = H.get(n - 1, j);
          H.set(n - 1, j, q * z + p * H.get(n, j));
          H.set(n, j, q * H.get(n, j) - p * z);
        }

        for (i = 0; i <= n; i++) {
          z = H.get(i, n - 1);
          H.set(i, n - 1, q * z + p * H.get(i, n));
          H.set(i, n, q * H.get(i, n) - p * z);
        }

        for (i = low; i <= high; i++) {
          z = V.get(i, n - 1);
          V.set(i, n - 1, q * z + p * V.get(i, n));
          V.set(i, n, q * V.get(i, n) - p * z);
        }
      } else {
        d[n - 1] = x + p;
        d[n] = x + p;
        e[n - 1] = z;
        e[n] = -z;
      }

      n = n - 2;
      iter = 0;
    } else {
      x = H.get(n, n);
      y = 0;
      w = 0;
      if (l < n) {
        y = H.get(n - 1, n - 1);
        w = H.get(n, n - 1) * H.get(n - 1, n);
      }

      if (iter === 10) {
        exshift += x;
        for (i = low; i <= n; i++) {
          H.set(i, i, H.get(i, i) - x);
        }
        s = Math.abs(H.get(n, n - 1)) + Math.abs(H.get(n - 1, n - 2));
        x = y = 0.75 * s;
        w = -0.4375 * s * s;
      }

      if (iter === 30) {
        s = (y - x) / 2;
        s = s * s + w;
        if (s > 0) {
          s = Math.sqrt(s);
          if (y < x) {
            s = -s;
          }
          s = x - w / ((y - x) / 2 + s);
          for (i = low; i <= n; i++) {
            H.set(i, i, H.get(i, i) - s);
          }
          exshift += s;
          x = y = w = 0.964;
        }
      }

      iter = iter + 1;

      m = n - 2;
      while (m >= l) {
        z = H.get(m, m);
        r = x - z;
        s = y - z;
        p = (r * s - w) / H.get(m + 1, m) + H.get(m, m + 1);
        q = H.get(m + 1, m + 1) - z - r - s;
        r = H.get(m + 2, m + 1);
        s = Math.abs(p) + Math.abs(q) + Math.abs(r);
        p = p / s;
        q = q / s;
        r = r / s;
        if (m === l) {
          break;
        }
        if (
          Math.abs(H.get(m, m - 1)) * (Math.abs(q) + Math.abs(r)) <
          eps *
            (Math.abs(p) *
              (Math.abs(H.get(m - 1, m - 1)) +
                Math.abs(z) +
                Math.abs(H.get(m + 1, m + 1))))
        ) {
          break;
        }
        m--;
      }

      for (i = m + 2; i <= n; i++) {
        H.set(i, i - 2, 0);
        if (i > m + 2) {
          H.set(i, i - 3, 0);
        }
      }

      for (k = m; k <= n - 1; k++) {
        notlast = k !== n - 1;
        if (k !== m) {
          p = H.get(k, k - 1);
          q = H.get(k + 1, k - 1);
          r = notlast ? H.get(k + 2, k - 1) : 0;
          x = Math.abs(p) + Math.abs(q) + Math.abs(r);
          if (x !== 0) {
            p = p / x;
            q = q / x;
            r = r / x;
          }
        }

        if (x === 0) {
          break;
        }

        s = Math.sqrt(p * p + q * q + r * r);
        if (p < 0) {
          s = -s;
        }

        if (s !== 0) {
          if (k !== m) {
            H.set(k, k - 1, -s * x);
          } else if (l !== m) {
            H.set(k, k - 1, -H.get(k, k - 1));
          }

          p = p + s;
          x = p / s;
          y = q / s;
          z = r / s;
          q = q / p;
          r = r / p;

          for (j = k; j < nn; j++) {
            p = H.get(k, j) + q * H.get(k + 1, j);
            if (notlast) {
              p = p + r * H.get(k + 2, j);
              H.set(k + 2, j, H.get(k + 2, j) - p * z);
            }

            H.set(k, j, H.get(k, j) - p * x);
            H.set(k + 1, j, H.get(k + 1, j) - p * y);
          }

          for (i = 0; i <= Math.min(n, k + 3); i++) {
            p = x * H.get(i, k) + y * H.get(i, k + 1);
            if (notlast) {
              p = p + z * H.get(i, k + 2);
              H.set(i, k + 2, H.get(i, k + 2) - p * r);
            }

            H.set(i, k, H.get(i, k) - p);
            H.set(i, k + 1, H.get(i, k + 1) - p * q);
          }

          for (i = low; i <= high; i++) {
            p = x * V.get(i, k) + y * V.get(i, k + 1);
            if (notlast) {
              p = p + z * V.get(i, k + 2);
              V.set(i, k + 2, V.get(i, k + 2) - p * r);
            }

            V.set(i, k, V.get(i, k) - p);
            V.set(i, k + 1, V.get(i, k + 1) - p * q);
          }
        }
      }
    }
  }

  if (norm === 0) {
    return;
  }

  for (n = nn - 1; n >= 0; n--) {
    p = d[n];
    q = e[n];

    if (q === 0) {
      l = n;
      H.set(n, n, 1);
      for (i = n - 1; i >= 0; i--) {
        w = H.get(i, i) - p;
        r = 0;
        for (j = l; j <= n; j++) {
          r = r + H.get(i, j) * H.get(j, n);
        }

        if (e[i] < 0) {
          z = w;
          s = r;
        } else {
          l = i;
          if (e[i] === 0) {
            H.set(i, n, w !== 0 ? -r / w : -r / (eps * norm));
          } else {
            x = H.get(i, i + 1);
            y = H.get(i + 1, i);
            q = (d[i] - p) * (d[i] - p) + e[i] * e[i];
            t = (x * s - z * r) / q;
            H.set(i, n, t);
            H.set(
              i + 1,
              n,
              Math.abs(x) > Math.abs(z) ? (-r - w * t) / x : (-s - y * t) / z,
            );
          }

          t = Math.abs(H.get(i, n));
          if (eps * t * t > 1) {
            for (j = i; j <= n; j++) {
              H.set(j, n, H.get(j, n) / t);
            }
          }
        }
      }
    } else if (q < 0) {
      l = n - 1;

      if (Math.abs(H.get(n, n - 1)) > Math.abs(H.get(n - 1, n))) {
        H.set(n - 1, n - 1, q / H.get(n, n - 1));
        H.set(n - 1, n, -(H.get(n, n) - p) / H.get(n, n - 1));
      } else {
        cdivres = cdiv(0, -H.get(n - 1, n), H.get(n - 1, n - 1) - p, q);
        H.set(n - 1, n - 1, cdivres[0]);
        H.set(n - 1, n, cdivres[1]);
      }

      H.set(n, n - 1, 0);
      H.set(n, n, 1);
      for (i = n - 2; i >= 0; i--) {
        ra = 0;
        sa = 0;
        for (j = l; j <= n; j++) {
          ra = ra + H.get(i, j) * H.get(j, n - 1);
          sa = sa + H.get(i, j) * H.get(j, n);
        }

        w = H.get(i, i) - p;

        if (e[i] < 0) {
          z = w;
          r = ra;
          s = sa;
        } else {
          l = i;
          if (e[i] === 0) {
            cdivres = cdiv(-ra, -sa, w, q);
            H.set(i, n - 1, cdivres[0]);
            H.set(i, n, cdivres[1]);
          } else {
            x = H.get(i, i + 1);
            y = H.get(i + 1, i);
            vr = (d[i] - p) * (d[i] - p) + e[i] * e[i] - q * q;
            vi = (d[i] - p) * 2 * q;
            if (vr === 0 && vi === 0) {
              vr =
                eps *
                norm *
                (Math.abs(w) +
                  Math.abs(q) +
                  Math.abs(x) +
                  Math.abs(y) +
                  Math.abs(z));
            }
            cdivres = cdiv(
              x * r - z * ra + q * sa,
              x * s - z * sa - q * ra,
              vr,
              vi,
            );
            H.set(i, n - 1, cdivres[0]);
            H.set(i, n, cdivres[1]);
            if (Math.abs(x) > Math.abs(z) + Math.abs(q)) {
              H.set(
                i + 1,
                n - 1,
                (-ra - w * H.get(i, n - 1) + q * H.get(i, n)) / x,
              );
              H.set(
                i + 1,
                n,
                (-sa - w * H.get(i, n) - q * H.get(i, n - 1)) / x,
              );
            } else {
              cdivres = cdiv(
                -r - y * H.get(i, n - 1),
                -s - y * H.get(i, n),
                z,
                q,
              );
              H.set(i + 1, n - 1, cdivres[0]);
              H.set(i + 1, n, cdivres[1]);
            }
          }

          t = Math.max(Math.abs(H.get(i, n - 1)), Math.abs(H.get(i, n)));
          if (eps * t * t > 1) {
            for (j = i; j <= n; j++) {
              H.set(j, n - 1, H.get(j, n - 1) / t);
              H.set(j, n, H.get(j, n) / t);
            }
          }
        }
      }
    }
  }

  for (i = 0; i < nn; i++) {
    if (i < low || i > high) {
      for (j = i; j < nn; j++) {
        V.set(i, j, H.get(i, j));
      }
    }
  }

  for (j = nn - 1; j >= low; j--) {
    for (i = low; i <= high; i++) {
      z = 0;
      for (k = low; k <= Math.min(j, high); k++) {
        z = z + V.get(i, k) * H.get(k, j);
      }
      V.set(i, j, z);
    }
  }
}

function cdiv(xr, xi, yr, yi) {
  let r, d;
  if (Math.abs(yr) > Math.abs(yi)) {
    r = yi / yr;
    d = yr + r * yi;
    return [(xr + r * xi) / d, (xi - r * xr) / d];
  } else {
    r = yr / yi;
    d = yi + r * yr;
    return [(r * xr + xi) / d, (r * xi - xr) / d];
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/dc/lu.js":
/*!*********************************************!*\
  !*** ./node_modules/ml-matrix/src/dc/lu.js ***!
  \*********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ LuDecomposition)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");



class LuDecomposition {
  constructor(matrix) {
    matrix = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(matrix);

    let lu = matrix.clone();
    let rows = lu.rows;
    let columns = lu.columns;
    let pivotVector = new Float64Array(rows);
    let pivotSign = 1;
    let i, j, k, p, s, t, v;
    let LUcolj, kmax;

    for (i = 0; i < rows; i++) {
      pivotVector[i] = i;
    }

    LUcolj = new Float64Array(rows);

    for (j = 0; j < columns; j++) {
      for (i = 0; i < rows; i++) {
        LUcolj[i] = lu.get(i, j);
      }

      for (i = 0; i < rows; i++) {
        kmax = Math.min(i, j);
        s = 0;
        for (k = 0; k < kmax; k++) {
          s += lu.get(i, k) * LUcolj[k];
        }
        LUcolj[i] -= s;
        lu.set(i, j, LUcolj[i]);
      }

      p = j;
      for (i = j + 1; i < rows; i++) {
        if (Math.abs(LUcolj[i]) > Math.abs(LUcolj[p])) {
          p = i;
        }
      }

      if (p !== j) {
        for (k = 0; k < columns; k++) {
          t = lu.get(p, k);
          lu.set(p, k, lu.get(j, k));
          lu.set(j, k, t);
        }

        v = pivotVector[p];
        pivotVector[p] = pivotVector[j];
        pivotVector[j] = v;

        pivotSign = -pivotSign;
      }

      if (j < rows && lu.get(j, j) !== 0) {
        for (i = j + 1; i < rows; i++) {
          lu.set(i, j, lu.get(i, j) / lu.get(j, j));
        }
      }
    }

    this.LU = lu;
    this.pivotVector = pivotVector;
    this.pivotSign = pivotSign;
  }

  isSingular() {
    let data = this.LU;
    let col = data.columns;
    for (let j = 0; j < col; j++) {
      if (data.get(j, j) === 0) {
        return true;
      }
    }
    return false;
  }

  solve(value) {
    value = _matrix__WEBPACK_IMPORTED_MODULE_1__["default"].checkMatrix(value);

    let lu = this.LU;
    let rows = lu.rows;

    if (rows !== value.rows) {
      throw new Error('Invalid matrix dimensions');
    }
    if (this.isSingular()) {
      throw new Error('LU matrix is singular');
    }

    let count = value.columns;
    let X = value.subMatrixRow(this.pivotVector, 0, count - 1);
    let columns = lu.columns;
    let i, j, k;

    for (k = 0; k < columns; k++) {
      for (i = k + 1; i < columns; i++) {
        for (j = 0; j < count; j++) {
          X.set(i, j, X.get(i, j) - X.get(k, j) * lu.get(i, k));
        }
      }
    }
    for (k = columns - 1; k >= 0; k--) {
      for (j = 0; j < count; j++) {
        X.set(k, j, X.get(k, j) / lu.get(k, k));
      }
      for (i = 0; i < k; i++) {
        for (j = 0; j < count; j++) {
          X.set(i, j, X.get(i, j) - X.get(k, j) * lu.get(i, k));
        }
      }
    }
    return X;
  }

  get determinant() {
    let data = this.LU;
    if (!data.isSquare()) {
      throw new Error('Matrix must be square');
    }
    let determinant = this.pivotSign;
    let col = data.columns;
    for (let j = 0; j < col; j++) {
      determinant *= data.get(j, j);
    }
    return determinant;
  }

  get lowerTriangularMatrix() {
    let data = this.LU;
    let rows = data.rows;
    let columns = data.columns;
    let X = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](rows, columns);
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < columns; j++) {
        if (i > j) {
          X.set(i, j, data.get(i, j));
        } else if (i === j) {
          X.set(i, j, 1);
        } else {
          X.set(i, j, 0);
        }
      }
    }
    return X;
  }

  get upperTriangularMatrix() {
    let data = this.LU;
    let rows = data.rows;
    let columns = data.columns;
    let X = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](rows, columns);
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < columns; j++) {
        if (i <= j) {
          X.set(i, j, data.get(i, j));
        } else {
          X.set(i, j, 0);
        }
      }
    }
    return X;
  }

  get pivotPermutationVector() {
    return Array.from(this.pivotVector);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/dc/nipals.js":
/*!*************************************************!*\
  !*** ./node_modules/ml-matrix/src/dc/nipals.js ***!
  \*************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ nipals)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");





class nipals {
  constructor(X, options = {}) {
    X = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_1__["default"].checkMatrix(X);
    let { Y } = options;
    const {
      scaleScores = false,
      maxIterations = 1000,
      terminationCriteria = 1e-10,
    } = options;

    let u;
    if (Y) {
      if ((0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(Y) && typeof Y[0] === 'number') {
        Y = _matrix__WEBPACK_IMPORTED_MODULE_2__["default"].columnVector(Y);
      } else {
        Y = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_1__["default"].checkMatrix(Y);
      }
      if (Y.rows !== X.rows) {
        throw new Error('Y should have the same number of rows as X');
      }
      u = Y.getColumnVector(0);
    } else {
      u = X.getColumnVector(0);
    }

    let diff = 1;
    let t, q, w, tOld;

    for (
      let counter = 0;
      counter < maxIterations && diff > terminationCriteria;
      counter++
    ) {
      w = X.transpose().mmul(u).div(u.transpose().mmul(u).get(0, 0));
      w = w.div(w.norm());

      t = X.mmul(w).div(w.transpose().mmul(w).get(0, 0));

      if (counter > 0) {
        diff = t.clone().sub(tOld).pow(2).sum();
      }
      tOld = t.clone();

      if (Y) {
        q = Y.transpose().mmul(t).div(t.transpose().mmul(t).get(0, 0));
        q = q.div(q.norm());

        u = Y.mmul(q).div(q.transpose().mmul(q).get(0, 0));
      } else {
        u = t;
      }
    }

    if (Y) {
      let p = X.transpose().mmul(t).div(t.transpose().mmul(t).get(0, 0));
      p = p.div(p.norm());
      let xResidual = X.clone().sub(t.clone().mmul(p.transpose()));
      let residual = u.transpose().mmul(t).div(t.transpose().mmul(t).get(0, 0));
      let yResidual = Y.clone().sub(
        t.clone().mulS(residual.get(0, 0)).mmul(q.transpose()),
      );

      this.t = t;
      this.p = p.transpose();
      this.w = w.transpose();
      this.q = q;
      this.u = u;
      this.s = t.transpose().mmul(t);
      this.xResidual = xResidual;
      this.yResidual = yResidual;
      this.betas = residual;
    } else {
      this.w = w.transpose();
      this.s = t.transpose().mmul(t).sqrt();
      if (scaleScores) {
        this.t = t.clone().div(this.s.get(0, 0));
      } else {
        this.t = t;
      }
      this.xResidual = X.sub(t.mmul(w.transpose()));
    }
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/dc/qr.js":
/*!*********************************************!*\
  !*** ./node_modules/ml-matrix/src/dc/qr.js ***!
  \*********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ QrDecomposition)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./util */ "./node_modules/ml-matrix/src/dc/util.js");





class QrDecomposition {
  constructor(value) {
    value = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(value);

    let qr = value.clone();
    let m = value.rows;
    let n = value.columns;
    let rdiag = new Float64Array(n);
    let i, j, k, s;

    for (k = 0; k < n; k++) {
      let nrm = 0;
      for (i = k; i < m; i++) {
        nrm = (0,_util__WEBPACK_IMPORTED_MODULE_1__.hypotenuse)(nrm, qr.get(i, k));
      }
      if (nrm !== 0) {
        if (qr.get(k, k) < 0) {
          nrm = -nrm;
        }
        for (i = k; i < m; i++) {
          qr.set(i, k, qr.get(i, k) / nrm);
        }
        qr.set(k, k, qr.get(k, k) + 1);
        for (j = k + 1; j < n; j++) {
          s = 0;
          for (i = k; i < m; i++) {
            s += qr.get(i, k) * qr.get(i, j);
          }
          s = -s / qr.get(k, k);
          for (i = k; i < m; i++) {
            qr.set(i, j, qr.get(i, j) + s * qr.get(i, k));
          }
        }
      }
      rdiag[k] = -nrm;
    }

    this.QR = qr;
    this.Rdiag = rdiag;
  }

  solve(value) {
    value = _matrix__WEBPACK_IMPORTED_MODULE_2__["default"].checkMatrix(value);

    let qr = this.QR;
    let m = qr.rows;

    if (value.rows !== m) {
      throw new Error('Matrix row dimensions must agree');
    }
    if (!this.isFullRank()) {
      throw new Error('Matrix is rank deficient');
    }

    let count = value.columns;
    let X = value.clone();
    let n = qr.columns;
    let i, j, k, s;

    for (k = 0; k < n; k++) {
      for (j = 0; j < count; j++) {
        s = 0;
        for (i = k; i < m; i++) {
          s += qr.get(i, k) * X.get(i, j);
        }
        s = -s / qr.get(k, k);
        for (i = k; i < m; i++) {
          X.set(i, j, X.get(i, j) + s * qr.get(i, k));
        }
      }
    }
    for (k = n - 1; k >= 0; k--) {
      for (j = 0; j < count; j++) {
        X.set(k, j, X.get(k, j) / this.Rdiag[k]);
      }
      for (i = 0; i < k; i++) {
        for (j = 0; j < count; j++) {
          X.set(i, j, X.get(i, j) - X.get(k, j) * qr.get(i, k));
        }
      }
    }

    return X.subMatrix(0, n - 1, 0, count - 1);
  }

  isFullRank() {
    let columns = this.QR.columns;
    for (let i = 0; i < columns; i++) {
      if (this.Rdiag[i] === 0) {
        return false;
      }
    }
    return true;
  }

  get upperTriangularMatrix() {
    let qr = this.QR;
    let n = qr.columns;
    let X = new _matrix__WEBPACK_IMPORTED_MODULE_2__["default"](n, n);
    let i, j;
    for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++) {
        if (i < j) {
          X.set(i, j, qr.get(i, j));
        } else if (i === j) {
          X.set(i, j, this.Rdiag[i]);
        } else {
          X.set(i, j, 0);
        }
      }
    }
    return X;
  }

  get orthogonalMatrix() {
    let qr = this.QR;
    let rows = qr.rows;
    let columns = qr.columns;
    let X = new _matrix__WEBPACK_IMPORTED_MODULE_2__["default"](rows, columns);
    let i, j, k, s;

    for (k = columns - 1; k >= 0; k--) {
      for (i = 0; i < rows; i++) {
        X.set(i, k, 0);
      }
      X.set(k, k, 1);
      for (j = k; j < columns; j++) {
        if (qr.get(k, k) !== 0) {
          s = 0;
          for (i = k; i < rows; i++) {
            s += qr.get(i, k) * X.get(i, j);
          }

          s = -s / qr.get(k, k);

          for (i = k; i < rows; i++) {
            X.set(i, j, X.get(i, j) + s * qr.get(i, k));
          }
        }
      }
    }
    return X;
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/dc/svd.js":
/*!**********************************************!*\
  !*** ./node_modules/ml-matrix/src/dc/svd.js ***!
  \**********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ SingularValueDecomposition)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./util */ "./node_modules/ml-matrix/src/dc/util.js");





class SingularValueDecomposition {
  constructor(value, options = {}) {
    value = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(value);

    if (value.isEmpty()) {
      throw new Error('Matrix must be non-empty');
    }

    let m = value.rows;
    let n = value.columns;

    const {
      computeLeftSingularVectors = true,
      computeRightSingularVectors = true,
      autoTranspose = false,
    } = options;

    let wantu = Boolean(computeLeftSingularVectors);
    let wantv = Boolean(computeRightSingularVectors);

    let swapped = false;
    let a;
    if (m < n) {
      if (!autoTranspose) {
        a = value.clone();
        // eslint-disable-next-line no-console
        console.warn(
          'Computing SVD on a matrix with more columns than rows. Consider enabling autoTranspose',
        );
      } else {
        a = value.transpose();
        m = a.rows;
        n = a.columns;
        swapped = true;
        let aux = wantu;
        wantu = wantv;
        wantv = aux;
      }
    } else {
      a = value.clone();
    }

    let nu = Math.min(m, n);
    let ni = Math.min(m + 1, n);
    let s = new Float64Array(ni);
    let U = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](m, nu);
    let V = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](n, n);

    let e = new Float64Array(n);
    let work = new Float64Array(m);

    let si = new Float64Array(ni);
    for (let i = 0; i < ni; i++) si[i] = i;

    let nct = Math.min(m - 1, n);
    let nrt = Math.max(0, Math.min(n - 2, m));
    let mrc = Math.max(nct, nrt);

    for (let k = 0; k < mrc; k++) {
      if (k < nct) {
        s[k] = 0;
        for (let i = k; i < m; i++) {
          s[k] = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(s[k], a.get(i, k));
        }
        if (s[k] !== 0) {
          if (a.get(k, k) < 0) {
            s[k] = -s[k];
          }
          for (let i = k; i < m; i++) {
            a.set(i, k, a.get(i, k) / s[k]);
          }
          a.set(k, k, a.get(k, k) + 1);
        }
        s[k] = -s[k];
      }

      for (let j = k + 1; j < n; j++) {
        if (k < nct && s[k] !== 0) {
          let t = 0;
          for (let i = k; i < m; i++) {
            t += a.get(i, k) * a.get(i, j);
          }
          t = -t / a.get(k, k);
          for (let i = k; i < m; i++) {
            a.set(i, j, a.get(i, j) + t * a.get(i, k));
          }
        }
        e[j] = a.get(k, j);
      }

      if (wantu && k < nct) {
        for (let i = k; i < m; i++) {
          U.set(i, k, a.get(i, k));
        }
      }

      if (k < nrt) {
        e[k] = 0;
        for (let i = k + 1; i < n; i++) {
          e[k] = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(e[k], e[i]);
        }
        if (e[k] !== 0) {
          if (e[k + 1] < 0) {
            e[k] = 0 - e[k];
          }
          for (let i = k + 1; i < n; i++) {
            e[i] /= e[k];
          }
          e[k + 1] += 1;
        }
        e[k] = -e[k];
        if (k + 1 < m && e[k] !== 0) {
          for (let i = k + 1; i < m; i++) {
            work[i] = 0;
          }
          for (let i = k + 1; i < m; i++) {
            for (let j = k + 1; j < n; j++) {
              work[i] += e[j] * a.get(i, j);
            }
          }
          for (let j = k + 1; j < n; j++) {
            let t = -e[j] / e[k + 1];
            for (let i = k + 1; i < m; i++) {
              a.set(i, j, a.get(i, j) + t * work[i]);
            }
          }
        }
        if (wantv) {
          for (let i = k + 1; i < n; i++) {
            V.set(i, k, e[i]);
          }
        }
      }
    }

    let p = Math.min(n, m + 1);
    if (nct < n) {
      s[nct] = a.get(nct, nct);
    }
    if (m < p) {
      s[p - 1] = 0;
    }
    if (nrt + 1 < p) {
      e[nrt] = a.get(nrt, p - 1);
    }
    e[p - 1] = 0;

    if (wantu) {
      for (let j = nct; j < nu; j++) {
        for (let i = 0; i < m; i++) {
          U.set(i, j, 0);
        }
        U.set(j, j, 1);
      }
      for (let k = nct - 1; k >= 0; k--) {
        if (s[k] !== 0) {
          for (let j = k + 1; j < nu; j++) {
            let t = 0;
            for (let i = k; i < m; i++) {
              t += U.get(i, k) * U.get(i, j);
            }
            t = -t / U.get(k, k);
            for (let i = k; i < m; i++) {
              U.set(i, j, U.get(i, j) + t * U.get(i, k));
            }
          }
          for (let i = k; i < m; i++) {
            U.set(i, k, -U.get(i, k));
          }
          U.set(k, k, 1 + U.get(k, k));
          for (let i = 0; i < k - 1; i++) {
            U.set(i, k, 0);
          }
        } else {
          for (let i = 0; i < m; i++) {
            U.set(i, k, 0);
          }
          U.set(k, k, 1);
        }
      }
    }

    if (wantv) {
      for (let k = n - 1; k >= 0; k--) {
        if (k < nrt && e[k] !== 0) {
          for (let j = k + 1; j < n; j++) {
            let t = 0;
            for (let i = k + 1; i < n; i++) {
              t += V.get(i, k) * V.get(i, j);
            }
            t = -t / V.get(k + 1, k);
            for (let i = k + 1; i < n; i++) {
              V.set(i, j, V.get(i, j) + t * V.get(i, k));
            }
          }
        }
        for (let i = 0; i < n; i++) {
          V.set(i, k, 0);
        }
        V.set(k, k, 1);
      }
    }

    let pp = p - 1;
    let iter = 0;
    let eps = Number.EPSILON;
    while (p > 0) {
      let k, kase;
      for (k = p - 2; k >= -1; k--) {
        if (k === -1) {
          break;
        }
        const alpha =
          Number.MIN_VALUE + eps * Math.abs(s[k] + Math.abs(s[k + 1]));
        if (Math.abs(e[k]) <= alpha || Number.isNaN(e[k])) {
          e[k] = 0;
          break;
        }
      }
      if (k === p - 2) {
        kase = 4;
      } else {
        let ks;
        for (ks = p - 1; ks >= k; ks--) {
          if (ks === k) {
            break;
          }
          let t =
            (ks !== p ? Math.abs(e[ks]) : 0) +
            (ks !== k + 1 ? Math.abs(e[ks - 1]) : 0);
          if (Math.abs(s[ks]) <= eps * t) {
            s[ks] = 0;
            break;
          }
        }
        if (ks === k) {
          kase = 3;
        } else if (ks === p - 1) {
          kase = 1;
        } else {
          kase = 2;
          k = ks;
        }
      }

      k++;

      switch (kase) {
        case 1: {
          let f = e[p - 2];
          e[p - 2] = 0;
          for (let j = p - 2; j >= k; j--) {
            let t = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(s[j], f);
            let cs = s[j] / t;
            let sn = f / t;
            s[j] = t;
            if (j !== k) {
              f = -sn * e[j - 1];
              e[j - 1] = cs * e[j - 1];
            }
            if (wantv) {
              for (let i = 0; i < n; i++) {
                t = cs * V.get(i, j) + sn * V.get(i, p - 1);
                V.set(i, p - 1, -sn * V.get(i, j) + cs * V.get(i, p - 1));
                V.set(i, j, t);
              }
            }
          }
          break;
        }
        case 2: {
          let f = e[k - 1];
          e[k - 1] = 0;
          for (let j = k; j < p; j++) {
            let t = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(s[j], f);
            let cs = s[j] / t;
            let sn = f / t;
            s[j] = t;
            f = -sn * e[j];
            e[j] = cs * e[j];
            if (wantu) {
              for (let i = 0; i < m; i++) {
                t = cs * U.get(i, j) + sn * U.get(i, k - 1);
                U.set(i, k - 1, -sn * U.get(i, j) + cs * U.get(i, k - 1));
                U.set(i, j, t);
              }
            }
          }
          break;
        }
        case 3: {
          const scale = Math.max(
            Math.abs(s[p - 1]),
            Math.abs(s[p - 2]),
            Math.abs(e[p - 2]),
            Math.abs(s[k]),
            Math.abs(e[k]),
          );
          const sp = s[p - 1] / scale;
          const spm1 = s[p - 2] / scale;
          const epm1 = e[p - 2] / scale;
          const sk = s[k] / scale;
          const ek = e[k] / scale;
          const b = ((spm1 + sp) * (spm1 - sp) + epm1 * epm1) / 2;
          const c = sp * epm1 * (sp * epm1);
          let shift = 0;
          if (b !== 0 || c !== 0) {
            if (b < 0) {
              shift = 0 - Math.sqrt(b * b + c);
            } else {
              shift = Math.sqrt(b * b + c);
            }
            shift = c / (b + shift);
          }
          let f = (sk + sp) * (sk - sp) + shift;
          let g = sk * ek;
          for (let j = k; j < p - 1; j++) {
            let t = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(f, g);
            if (t === 0) t = Number.MIN_VALUE;
            let cs = f / t;
            let sn = g / t;
            if (j !== k) {
              e[j - 1] = t;
            }
            f = cs * s[j] + sn * e[j];
            e[j] = cs * e[j] - sn * s[j];
            g = sn * s[j + 1];
            s[j + 1] = cs * s[j + 1];
            if (wantv) {
              for (let i = 0; i < n; i++) {
                t = cs * V.get(i, j) + sn * V.get(i, j + 1);
                V.set(i, j + 1, -sn * V.get(i, j) + cs * V.get(i, j + 1));
                V.set(i, j, t);
              }
            }
            t = (0,_util__WEBPACK_IMPORTED_MODULE_2__.hypotenuse)(f, g);
            if (t === 0) t = Number.MIN_VALUE;
            cs = f / t;
            sn = g / t;
            s[j] = t;
            f = cs * e[j] + sn * s[j + 1];
            s[j + 1] = -sn * e[j] + cs * s[j + 1];
            g = sn * e[j + 1];
            e[j + 1] = cs * e[j + 1];
            if (wantu && j < m - 1) {
              for (let i = 0; i < m; i++) {
                t = cs * U.get(i, j) + sn * U.get(i, j + 1);
                U.set(i, j + 1, -sn * U.get(i, j) + cs * U.get(i, j + 1));
                U.set(i, j, t);
              }
            }
          }
          e[p - 2] = f;
          iter = iter + 1;
          break;
        }
        case 4: {
          if (s[k] <= 0) {
            s[k] = s[k] < 0 ? -s[k] : 0;
            if (wantv) {
              for (let i = 0; i <= pp; i++) {
                V.set(i, k, -V.get(i, k));
              }
            }
          }
          while (k < pp) {
            if (s[k] >= s[k + 1]) {
              break;
            }
            let t = s[k];
            s[k] = s[k + 1];
            s[k + 1] = t;
            if (wantv && k < n - 1) {
              for (let i = 0; i < n; i++) {
                t = V.get(i, k + 1);
                V.set(i, k + 1, V.get(i, k));
                V.set(i, k, t);
              }
            }
            if (wantu && k < m - 1) {
              for (let i = 0; i < m; i++) {
                t = U.get(i, k + 1);
                U.set(i, k + 1, U.get(i, k));
                U.set(i, k, t);
              }
            }
            k++;
          }
          iter = 0;
          p--;
          break;
        }
        // no default
      }
    }

    if (swapped) {
      let tmp = V;
      V = U;
      U = tmp;
    }

    this.m = m;
    this.n = n;
    this.s = s;
    this.U = U;
    this.V = V;
  }

  solve(value) {
    let Y = value;
    let e = this.threshold;
    let scols = this.s.length;
    let Ls = _matrix__WEBPACK_IMPORTED_MODULE_1__["default"].zeros(scols, scols);

    for (let i = 0; i < scols; i++) {
      if (Math.abs(this.s[i]) <= e) {
        Ls.set(i, i, 0);
      } else {
        Ls.set(i, i, 1 / this.s[i]);
      }
    }

    let U = this.U;
    let V = this.rightSingularVectors;

    let VL = V.mmul(Ls);
    let vrows = V.rows;
    let urows = U.rows;
    let VLU = _matrix__WEBPACK_IMPORTED_MODULE_1__["default"].zeros(vrows, urows);

    for (let i = 0; i < vrows; i++) {
      for (let j = 0; j < urows; j++) {
        let sum = 0;
        for (let k = 0; k < scols; k++) {
          sum += VL.get(i, k) * U.get(j, k);
        }
        VLU.set(i, j, sum);
      }
    }

    return VLU.mmul(Y);
  }

  solveForDiagonal(value) {
    return this.solve(_matrix__WEBPACK_IMPORTED_MODULE_1__["default"].diag(value));
  }

  inverse() {
    let V = this.V;
    let e = this.threshold;
    let vrows = V.rows;
    let vcols = V.columns;
    let X = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](vrows, this.s.length);

    for (let i = 0; i < vrows; i++) {
      for (let j = 0; j < vcols; j++) {
        if (Math.abs(this.s[j]) > e) {
          X.set(i, j, V.get(i, j) / this.s[j]);
        }
      }
    }

    let U = this.U;

    let urows = U.rows;
    let ucols = U.columns;
    let Y = new _matrix__WEBPACK_IMPORTED_MODULE_1__["default"](vrows, urows);

    for (let i = 0; i < vrows; i++) {
      for (let j = 0; j < urows; j++) {
        let sum = 0;
        for (let k = 0; k < ucols; k++) {
          sum += X.get(i, k) * U.get(j, k);
        }
        Y.set(i, j, sum);
      }
    }

    return Y;
  }

  get condition() {
    return this.s[0] / this.s[Math.min(this.m, this.n) - 1];
  }

  get norm2() {
    return this.s[0];
  }

  get rank() {
    let tol = Math.max(this.m, this.n) * this.s[0] * Number.EPSILON;
    let r = 0;
    let s = this.s;
    for (let i = 0, ii = s.length; i < ii; i++) {
      if (s[i] > tol) {
        r++;
      }
    }
    return r;
  }

  get diagonal() {
    return Array.from(this.s);
  }

  get threshold() {
    return (Number.EPSILON / 2) * Math.max(this.m, this.n) * this.s[0];
  }

  get leftSingularVectors() {
    return this.U;
  }

  get rightSingularVectors() {
    return this.V;
  }

  get diagonalMatrix() {
    return _matrix__WEBPACK_IMPORTED_MODULE_1__["default"].diag(this.s);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/dc/util.js":
/*!***********************************************!*\
  !*** ./node_modules/ml-matrix/src/dc/util.js ***!
  \***********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "hypotenuse": () => (/* binding */ hypotenuse)
/* harmony export */ });
function hypotenuse(a, b) {
  let r = 0;
  if (Math.abs(a) > Math.abs(b)) {
    r = b / a;
    return Math.abs(a) * Math.sqrt(1 + r * r);
  }
  if (b !== 0) {
    r = a / b;
    return Math.abs(b) * Math.sqrt(1 + r * r);
  }
  return 0;
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/decompositions.js":
/*!******************************************************!*\
  !*** ./node_modules/ml-matrix/src/decompositions.js ***!
  \******************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "inverse": () => (/* binding */ inverse),
/* harmony export */   "solve": () => (/* binding */ solve)
/* harmony export */ });
/* harmony import */ var _dc_lu__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./dc/lu */ "./node_modules/ml-matrix/src/dc/lu.js");
/* harmony import */ var _dc_qr__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./dc/qr */ "./node_modules/ml-matrix/src/dc/qr.js");
/* harmony import */ var _dc_svd__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./dc/svd */ "./node_modules/ml-matrix/src/dc/svd.js");
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");






function inverse(matrix, useSVD = false) {
  matrix = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(matrix);
  if (useSVD) {
    return new _dc_svd__WEBPACK_IMPORTED_MODULE_1__["default"](matrix).inverse();
  } else {
    return solve(matrix, _matrix__WEBPACK_IMPORTED_MODULE_2__["default"].eye(matrix.rows));
  }
}

function solve(leftHandSide, rightHandSide, useSVD = false) {
  leftHandSide = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(leftHandSide);
  rightHandSide = _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(rightHandSide);
  if (useSVD) {
    return new _dc_svd__WEBPACK_IMPORTED_MODULE_1__["default"](leftHandSide).solve(rightHandSide);
  } else {
    return leftHandSide.isSquare()
      ? new _dc_lu__WEBPACK_IMPORTED_MODULE_3__["default"](leftHandSide).solve(rightHandSide)
      : new _dc_qr__WEBPACK_IMPORTED_MODULE_4__["default"](leftHandSide).solve(rightHandSide);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/determinant.js":
/*!***************************************************!*\
  !*** ./node_modules/ml-matrix/src/determinant.js ***!
  \***************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "determinant": () => (/* binding */ determinant)
/* harmony export */ });
/* harmony import */ var _dc_lu__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./dc/lu */ "./node_modules/ml-matrix/src/dc/lu.js");
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _views_selection__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./views/selection */ "./node_modules/ml-matrix/src/views/selection.js");




function determinant(matrix) {
  matrix = _matrix__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(matrix);
  if (matrix.isSquare()) {
    if (matrix.columns === 0) {
      return 1;
    }

    let a, b, c, d;
    if (matrix.columns === 2) {
      // 2 x 2 matrix
      a = matrix.get(0, 0);
      b = matrix.get(0, 1);
      c = matrix.get(1, 0);
      d = matrix.get(1, 1);

      return a * d - b * c;
    } else if (matrix.columns === 3) {
      // 3 x 3 matrix
      let subMatrix0, subMatrix1, subMatrix2;
      subMatrix0 = new _views_selection__WEBPACK_IMPORTED_MODULE_1__["default"](matrix, [1, 2], [1, 2]);
      subMatrix1 = new _views_selection__WEBPACK_IMPORTED_MODULE_1__["default"](matrix, [1, 2], [0, 2]);
      subMatrix2 = new _views_selection__WEBPACK_IMPORTED_MODULE_1__["default"](matrix, [1, 2], [0, 1]);
      a = matrix.get(0, 0);
      b = matrix.get(0, 1);
      c = matrix.get(0, 2);

      return (
        a * determinant(subMatrix0) -
        b * determinant(subMatrix1) +
        c * determinant(subMatrix2)
      );
    } else {
      // general purpose determinant using the LU decomposition
      return new _dc_lu__WEBPACK_IMPORTED_MODULE_2__["default"](matrix).determinant;
    }
  } else {
    throw Error('determinant can only be calculated for a square matrix');
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/index.js":
/*!*********************************************!*\
  !*** ./node_modules/ml-matrix/src/index.js ***!
  \*********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "AbstractMatrix": () => (/* reexport safe */ _matrix__WEBPACK_IMPORTED_MODULE_0__.AbstractMatrix),
/* harmony export */   "CHO": () => (/* reexport safe */ _dc_cholesky_js__WEBPACK_IMPORTED_MODULE_13__["default"]),
/* harmony export */   "CholeskyDecomposition": () => (/* reexport safe */ _dc_cholesky_js__WEBPACK_IMPORTED_MODULE_13__["default"]),
/* harmony export */   "EVD": () => (/* reexport safe */ _dc_evd_js__WEBPACK_IMPORTED_MODULE_12__["default"]),
/* harmony export */   "EigenvalueDecomposition": () => (/* reexport safe */ _dc_evd_js__WEBPACK_IMPORTED_MODULE_12__["default"]),
/* harmony export */   "LU": () => (/* reexport safe */ _dc_lu_js__WEBPACK_IMPORTED_MODULE_14__["default"]),
/* harmony export */   "LuDecomposition": () => (/* reexport safe */ _dc_lu_js__WEBPACK_IMPORTED_MODULE_14__["default"]),
/* harmony export */   "Matrix": () => (/* reexport safe */ _matrix__WEBPACK_IMPORTED_MODULE_0__["default"]),
/* harmony export */   "MatrixColumnSelectionView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixColumnSelectionView),
/* harmony export */   "MatrixColumnView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixColumnView),
/* harmony export */   "MatrixFlipColumnView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixFlipColumnView),
/* harmony export */   "MatrixFlipRowView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixFlipRowView),
/* harmony export */   "MatrixRowSelectionView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixRowSelectionView),
/* harmony export */   "MatrixRowView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixRowView),
/* harmony export */   "MatrixSelectionView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixSelectionView),
/* harmony export */   "MatrixSubView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixSubView),
/* harmony export */   "MatrixTransposeView": () => (/* reexport safe */ _views_index__WEBPACK_IMPORTED_MODULE_1__.MatrixTransposeView),
/* harmony export */   "NIPALS": () => (/* reexport safe */ _dc_nipals_js__WEBPACK_IMPORTED_MODULE_16__["default"]),
/* harmony export */   "Nipals": () => (/* reexport safe */ _dc_nipals_js__WEBPACK_IMPORTED_MODULE_16__["default"]),
/* harmony export */   "QR": () => (/* reexport safe */ _dc_qr_js__WEBPACK_IMPORTED_MODULE_15__["default"]),
/* harmony export */   "QrDecomposition": () => (/* reexport safe */ _dc_qr_js__WEBPACK_IMPORTED_MODULE_15__["default"]),
/* harmony export */   "SVD": () => (/* reexport safe */ _dc_svd_js__WEBPACK_IMPORTED_MODULE_11__["default"]),
/* harmony export */   "SingularValueDecomposition": () => (/* reexport safe */ _dc_svd_js__WEBPACK_IMPORTED_MODULE_11__["default"]),
/* harmony export */   "WrapperMatrix1D": () => (/* reexport safe */ _wrap_WrapperMatrix1D__WEBPACK_IMPORTED_MODULE_3__["default"]),
/* harmony export */   "WrapperMatrix2D": () => (/* reexport safe */ _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_4__["default"]),
/* harmony export */   "correlation": () => (/* reexport safe */ _correlation__WEBPACK_IMPORTED_MODULE_10__.correlation),
/* harmony export */   "covariance": () => (/* reexport safe */ _covariance__WEBPACK_IMPORTED_MODULE_9__.covariance),
/* harmony export */   "default": () => (/* reexport safe */ _matrix__WEBPACK_IMPORTED_MODULE_0__["default"]),
/* harmony export */   "determinant": () => (/* reexport safe */ _determinant__WEBPACK_IMPORTED_MODULE_6__.determinant),
/* harmony export */   "inverse": () => (/* reexport safe */ _decompositions__WEBPACK_IMPORTED_MODULE_5__.inverse),
/* harmony export */   "linearDependencies": () => (/* reexport safe */ _linearDependencies__WEBPACK_IMPORTED_MODULE_7__.linearDependencies),
/* harmony export */   "pseudoInverse": () => (/* reexport safe */ _pseudoInverse__WEBPACK_IMPORTED_MODULE_8__.pseudoInverse),
/* harmony export */   "solve": () => (/* reexport safe */ _decompositions__WEBPACK_IMPORTED_MODULE_5__.solve),
/* harmony export */   "wrap": () => (/* reexport safe */ _wrap_wrap__WEBPACK_IMPORTED_MODULE_2__.wrap)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./matrix */ "./node_modules/ml-matrix/src/matrix.js");
/* harmony import */ var _views_index__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./views/index */ "./node_modules/ml-matrix/src/views/index.js");
/* harmony import */ var _wrap_wrap__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./wrap/wrap */ "./node_modules/ml-matrix/src/wrap/wrap.js");
/* harmony import */ var _wrap_WrapperMatrix1D__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./wrap/WrapperMatrix1D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix1D.js");
/* harmony import */ var _wrap_WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./wrap/WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");
/* harmony import */ var _decompositions__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ./decompositions */ "./node_modules/ml-matrix/src/decompositions.js");
/* harmony import */ var _determinant__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! ./determinant */ "./node_modules/ml-matrix/src/determinant.js");
/* harmony import */ var _linearDependencies__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! ./linearDependencies */ "./node_modules/ml-matrix/src/linearDependencies.js");
/* harmony import */ var _pseudoInverse__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! ./pseudoInverse */ "./node_modules/ml-matrix/src/pseudoInverse.js");
/* harmony import */ var _covariance__WEBPACK_IMPORTED_MODULE_9__ = __webpack_require__(/*! ./covariance */ "./node_modules/ml-matrix/src/covariance.js");
/* harmony import */ var _correlation__WEBPACK_IMPORTED_MODULE_10__ = __webpack_require__(/*! ./correlation */ "./node_modules/ml-matrix/src/correlation.js");
/* harmony import */ var _dc_svd_js__WEBPACK_IMPORTED_MODULE_11__ = __webpack_require__(/*! ./dc/svd.js */ "./node_modules/ml-matrix/src/dc/svd.js");
/* harmony import */ var _dc_evd_js__WEBPACK_IMPORTED_MODULE_12__ = __webpack_require__(/*! ./dc/evd.js */ "./node_modules/ml-matrix/src/dc/evd.js");
/* harmony import */ var _dc_cholesky_js__WEBPACK_IMPORTED_MODULE_13__ = __webpack_require__(/*! ./dc/cholesky.js */ "./node_modules/ml-matrix/src/dc/cholesky.js");
/* harmony import */ var _dc_lu_js__WEBPACK_IMPORTED_MODULE_14__ = __webpack_require__(/*! ./dc/lu.js */ "./node_modules/ml-matrix/src/dc/lu.js");
/* harmony import */ var _dc_qr_js__WEBPACK_IMPORTED_MODULE_15__ = __webpack_require__(/*! ./dc/qr.js */ "./node_modules/ml-matrix/src/dc/qr.js");
/* harmony import */ var _dc_nipals_js__WEBPACK_IMPORTED_MODULE_16__ = __webpack_require__(/*! ./dc/nipals.js */ "./node_modules/ml-matrix/src/dc/nipals.js");






















/***/ }),

/***/ "./node_modules/ml-matrix/src/inspect.js":
/*!***********************************************!*\
  !*** ./node_modules/ml-matrix/src/inspect.js ***!
  \***********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "inspectMatrix": () => (/* binding */ inspectMatrix),
/* harmony export */   "inspectMatrixWithOptions": () => (/* binding */ inspectMatrixWithOptions)
/* harmony export */ });
const indent = ' '.repeat(2);
const indentData = ' '.repeat(4);

function inspectMatrix() {
  return inspectMatrixWithOptions(this);
}

function inspectMatrixWithOptions(matrix, options = {}) {
  const {
    maxRows = 15,
    maxColumns = 10,
    maxNumSize = 8,
    padMinus = 'auto',
  } = options;
  return `${matrix.constructor.name} {
${indent}[
${indentData}${inspectData(matrix, maxRows, maxColumns, maxNumSize, padMinus)}
${indent}]
${indent}rows: ${matrix.rows}
${indent}columns: ${matrix.columns}
}`;
}

function inspectData(matrix, maxRows, maxColumns, maxNumSize, padMinus) {
  const { rows, columns } = matrix;
  const maxI = Math.min(rows, maxRows);
  const maxJ = Math.min(columns, maxColumns);
  const result = [];

  if (padMinus === 'auto') {
    padMinus = false;
    loop: for (let i = 0; i < maxI; i++) {
      for (let j = 0; j < maxJ; j++) {
        if (matrix.get(i, j) < 0) {
          padMinus = true;
          break loop;
        }
      }
    }
  }

  for (let i = 0; i < maxI; i++) {
    let line = [];
    for (let j = 0; j < maxJ; j++) {
      line.push(formatNumber(matrix.get(i, j), maxNumSize, padMinus));
    }
    result.push(`${line.join(' ')}`);
  }
  if (maxJ !== columns) {
    result[result.length - 1] += ` ... ${columns - maxColumns} more columns`;
  }
  if (maxI !== rows) {
    result.push(`... ${rows - maxRows} more rows`);
  }
  return result.join(`\n${indentData}`);
}

function formatNumber(num, maxNumSize, padMinus) {
  return (
    num >= 0 && padMinus
      ? ` ${formatNumber2(num, maxNumSize - 1)}`
      : formatNumber2(num, maxNumSize)
  ).padEnd(maxNumSize);
}

function formatNumber2(num, len) {
  // small.length numbers should be as is
  let str = num.toString();
  if (str.length <= len) return str;

  // (7)'0.00123' is better then (7)'1.23e-2'
  // (8)'0.000123' is worse then (7)'1.23e-3',
  let fix = num.toFixed(len);
  if (fix.length > len) {
    fix = num.toFixed(Math.max(0, len - (fix.length - len)));
  }
  if (
    fix.length <= len &&
    !fix.startsWith('0.000') &&
    !fix.startsWith('-0.000')
  ) {
    return fix;
  }

  // well, if it's still too long the user should've used longer numbers
  let exp = num.toExponential(len);
  if (exp.length > len) {
    exp = num.toExponential(Math.max(0, len - (exp.length - len)));
  }
  return exp.slice(0);
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/linearDependencies.js":
/*!**********************************************************!*\
  !*** ./node_modules/ml-matrix/src/linearDependencies.js ***!
  \**********************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "linearDependencies": () => (/* binding */ linearDependencies)
/* harmony export */ });
/* harmony import */ var _dc_svd__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./dc/svd */ "./node_modules/ml-matrix/src/dc/svd.js");
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./matrix */ "./node_modules/ml-matrix/src/matrix.js");



function xrange(n, exception) {
  let range = [];
  for (let i = 0; i < n; i++) {
    if (i !== exception) {
      range.push(i);
    }
  }
  return range;
}

function dependenciesOneRow(
  error,
  matrix,
  index,
  thresholdValue = 10e-10,
  thresholdError = 10e-10,
) {
  if (error > thresholdError) {
    return new Array(matrix.rows + 1).fill(0);
  } else {
    let returnArray = matrix.addRow(index, [0]);
    for (let i = 0; i < returnArray.rows; i++) {
      if (Math.abs(returnArray.get(i, 0)) < thresholdValue) {
        returnArray.set(i, 0, 0);
      }
    }
    return returnArray.to1DArray();
  }
}

function linearDependencies(matrix, options = {}) {
  const { thresholdValue = 10e-10, thresholdError = 10e-10 } = options;
  matrix = _matrix__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(matrix);

  let n = matrix.rows;
  let results = new _matrix__WEBPACK_IMPORTED_MODULE_0__["default"](n, n);

  for (let i = 0; i < n; i++) {
    let b = _matrix__WEBPACK_IMPORTED_MODULE_0__["default"].columnVector(matrix.getRow(i));
    let Abis = matrix.subMatrixRow(xrange(n, i)).transpose();
    let svd = new _dc_svd__WEBPACK_IMPORTED_MODULE_1__["default"](Abis);
    let x = svd.solve(b);
    let error = _matrix__WEBPACK_IMPORTED_MODULE_0__["default"].sub(b, Abis.mmul(x)).abs().max();
    results.setRow(
      i,
      dependenciesOneRow(error, x, i, thresholdValue, thresholdError),
    );
  }
  return results;
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/mathOperations.js":
/*!******************************************************!*\
  !*** ./node_modules/ml-matrix/src/mathOperations.js ***!
  \******************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "installMathOperations": () => (/* binding */ installMathOperations)
/* harmony export */ });
function installMathOperations(AbstractMatrix, Matrix) {
  AbstractMatrix.prototype.add = function add(value) {
    if (typeof value === 'number') return this.addS(value);
    return this.addM(value);
  };

  AbstractMatrix.prototype.addS = function addS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) + value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.addM = function addM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) + matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.add = function add(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.add(value);
  };

  AbstractMatrix.prototype.sub = function sub(value) {
    if (typeof value === 'number') return this.subS(value);
    return this.subM(value);
  };

  AbstractMatrix.prototype.subS = function subS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) - value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.subM = function subM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) - matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.sub = function sub(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.sub(value);
  };
  AbstractMatrix.prototype.subtract = AbstractMatrix.prototype.sub;
  AbstractMatrix.prototype.subtractS = AbstractMatrix.prototype.subS;
  AbstractMatrix.prototype.subtractM = AbstractMatrix.prototype.subM;
  AbstractMatrix.subtract = AbstractMatrix.sub;

  AbstractMatrix.prototype.mul = function mul(value) {
    if (typeof value === 'number') return this.mulS(value);
    return this.mulM(value);
  };

  AbstractMatrix.prototype.mulS = function mulS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) * value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.mulM = function mulM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) * matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.mul = function mul(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.mul(value);
  };
  AbstractMatrix.prototype.multiply = AbstractMatrix.prototype.mul;
  AbstractMatrix.prototype.multiplyS = AbstractMatrix.prototype.mulS;
  AbstractMatrix.prototype.multiplyM = AbstractMatrix.prototype.mulM;
  AbstractMatrix.multiply = AbstractMatrix.mul;

  AbstractMatrix.prototype.div = function div(value) {
    if (typeof value === 'number') return this.divS(value);
    return this.divM(value);
  };

  AbstractMatrix.prototype.divS = function divS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) / value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.divM = function divM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) / matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.div = function div(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.div(value);
  };
  AbstractMatrix.prototype.divide = AbstractMatrix.prototype.div;
  AbstractMatrix.prototype.divideS = AbstractMatrix.prototype.divS;
  AbstractMatrix.prototype.divideM = AbstractMatrix.prototype.divM;
  AbstractMatrix.divide = AbstractMatrix.div;

  AbstractMatrix.prototype.mod = function mod(value) {
    if (typeof value === 'number') return this.modS(value);
    return this.modM(value);
  };

  AbstractMatrix.prototype.modS = function modS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) % value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.modM = function modM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) % matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.mod = function mod(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.mod(value);
  };
  AbstractMatrix.prototype.modulus = AbstractMatrix.prototype.mod;
  AbstractMatrix.prototype.modulusS = AbstractMatrix.prototype.modS;
  AbstractMatrix.prototype.modulusM = AbstractMatrix.prototype.modM;
  AbstractMatrix.modulus = AbstractMatrix.mod;

  AbstractMatrix.prototype.and = function and(value) {
    if (typeof value === 'number') return this.andS(value);
    return this.andM(value);
  };

  AbstractMatrix.prototype.andS = function andS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) & value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.andM = function andM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) & matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.and = function and(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.and(value);
  };

  AbstractMatrix.prototype.or = function or(value) {
    if (typeof value === 'number') return this.orS(value);
    return this.orM(value);
  };

  AbstractMatrix.prototype.orS = function orS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) | value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.orM = function orM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) | matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.or = function or(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.or(value);
  };

  AbstractMatrix.prototype.xor = function xor(value) {
    if (typeof value === 'number') return this.xorS(value);
    return this.xorM(value);
  };

  AbstractMatrix.prototype.xorS = function xorS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) ^ value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.xorM = function xorM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) ^ matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.xor = function xor(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.xor(value);
  };

  AbstractMatrix.prototype.leftShift = function leftShift(value) {
    if (typeof value === 'number') return this.leftShiftS(value);
    return this.leftShiftM(value);
  };

  AbstractMatrix.prototype.leftShiftS = function leftShiftS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) << value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.leftShiftM = function leftShiftM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) << matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.leftShift = function leftShift(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.leftShift(value);
  };

  AbstractMatrix.prototype.signPropagatingRightShift = function signPropagatingRightShift(value) {
    if (typeof value === 'number') return this.signPropagatingRightShiftS(value);
    return this.signPropagatingRightShiftM(value);
  };

  AbstractMatrix.prototype.signPropagatingRightShiftS = function signPropagatingRightShiftS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) >> value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.signPropagatingRightShiftM = function signPropagatingRightShiftM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) >> matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.signPropagatingRightShift = function signPropagatingRightShift(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.signPropagatingRightShift(value);
  };

  AbstractMatrix.prototype.rightShift = function rightShift(value) {
    if (typeof value === 'number') return this.rightShiftS(value);
    return this.rightShiftM(value);
  };

  AbstractMatrix.prototype.rightShiftS = function rightShiftS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) >>> value);
      }
    }
    return this;
  };

  AbstractMatrix.prototype.rightShiftM = function rightShiftM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) >>> matrix.get(i, j));
      }
    }
    return this;
  };

  AbstractMatrix.rightShift = function rightShift(matrix, value) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.rightShift(value);
  };
  AbstractMatrix.prototype.zeroFillRightShift = AbstractMatrix.prototype.rightShift;
  AbstractMatrix.prototype.zeroFillRightShiftS = AbstractMatrix.prototype.rightShiftS;
  AbstractMatrix.prototype.zeroFillRightShiftM = AbstractMatrix.prototype.rightShiftM;
  AbstractMatrix.zeroFillRightShift = AbstractMatrix.rightShift;

  AbstractMatrix.prototype.not = function not() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, ~(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.not = function not(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.not();
  };

  AbstractMatrix.prototype.abs = function abs() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.abs(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.abs = function abs(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.abs();
  };

  AbstractMatrix.prototype.acos = function acos() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.acos(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.acos = function acos(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.acos();
  };

  AbstractMatrix.prototype.acosh = function acosh() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.acosh(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.acosh = function acosh(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.acosh();
  };

  AbstractMatrix.prototype.asin = function asin() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.asin(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.asin = function asin(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.asin();
  };

  AbstractMatrix.prototype.asinh = function asinh() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.asinh(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.asinh = function asinh(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.asinh();
  };

  AbstractMatrix.prototype.atan = function atan() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.atan(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.atan = function atan(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.atan();
  };

  AbstractMatrix.prototype.atanh = function atanh() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.atanh(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.atanh = function atanh(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.atanh();
  };

  AbstractMatrix.prototype.cbrt = function cbrt() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.cbrt(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.cbrt = function cbrt(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.cbrt();
  };

  AbstractMatrix.prototype.ceil = function ceil() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.ceil(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.ceil = function ceil(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.ceil();
  };

  AbstractMatrix.prototype.clz32 = function clz32() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.clz32(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.clz32 = function clz32(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.clz32();
  };

  AbstractMatrix.prototype.cos = function cos() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.cos(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.cos = function cos(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.cos();
  };

  AbstractMatrix.prototype.cosh = function cosh() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.cosh(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.cosh = function cosh(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.cosh();
  };

  AbstractMatrix.prototype.exp = function exp() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.exp(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.exp = function exp(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.exp();
  };

  AbstractMatrix.prototype.expm1 = function expm1() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.expm1(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.expm1 = function expm1(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.expm1();
  };

  AbstractMatrix.prototype.floor = function floor() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.floor(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.floor = function floor(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.floor();
  };

  AbstractMatrix.prototype.fround = function fround() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.fround(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.fround = function fround(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.fround();
  };

  AbstractMatrix.prototype.log = function log() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.log(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.log = function log(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.log();
  };

  AbstractMatrix.prototype.log1p = function log1p() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.log1p(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.log1p = function log1p(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.log1p();
  };

  AbstractMatrix.prototype.log10 = function log10() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.log10(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.log10 = function log10(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.log10();
  };

  AbstractMatrix.prototype.log2 = function log2() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.log2(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.log2 = function log2(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.log2();
  };

  AbstractMatrix.prototype.round = function round() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.round(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.round = function round(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.round();
  };

  AbstractMatrix.prototype.sign = function sign() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.sign(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.sign = function sign(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.sign();
  };

  AbstractMatrix.prototype.sin = function sin() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.sin(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.sin = function sin(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.sin();
  };

  AbstractMatrix.prototype.sinh = function sinh() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.sinh(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.sinh = function sinh(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.sinh();
  };

  AbstractMatrix.prototype.sqrt = function sqrt() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.sqrt(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.sqrt = function sqrt(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.sqrt();
  };

  AbstractMatrix.prototype.tan = function tan() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.tan(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.tan = function tan(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.tan();
  };

  AbstractMatrix.prototype.tanh = function tanh() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.tanh(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.tanh = function tanh(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.tanh();
  };

  AbstractMatrix.prototype.trunc = function trunc() {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.trunc(this.get(i, j)));
      }
    }
    return this;
  };

  AbstractMatrix.trunc = function trunc(matrix) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.trunc();
  };

  AbstractMatrix.pow = function pow(matrix, arg0) {
    const newMatrix = new Matrix(matrix);
    return newMatrix.pow(arg0);
  };

  AbstractMatrix.prototype.pow = function pow(value) {
    if (typeof value === 'number') return this.powS(value);
    return this.powM(value);
  };

  AbstractMatrix.prototype.powS = function powS(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.pow(this.get(i, j), value));
      }
    }
    return this;
  };

  AbstractMatrix.prototype.powM = function powM(matrix) {
    matrix = Matrix.checkMatrix(matrix);
    if (this.rows !== matrix.rows ||
      this.columns !== matrix.columns) {
      throw new RangeError('Matrices dimensions must be equal');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, Math.pow(this.get(i, j), matrix.get(i, j)));
      }
    }
    return this;
  };
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/matrix.js":
/*!**********************************************!*\
  !*** ./node_modules/ml-matrix/src/matrix.js ***!
  \**********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "AbstractMatrix": () => (/* binding */ AbstractMatrix),
/* harmony export */   "default": () => (/* binding */ Matrix)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");
/* harmony import */ var ml_array_rescale__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ml-array-rescale */ "./node_modules/ml-array-rescale/lib-es6/index.js");
/* harmony import */ var _inspect__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./inspect */ "./node_modules/ml-matrix/src/inspect.js");
/* harmony import */ var _mathOperations__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ./mathOperations */ "./node_modules/ml-matrix/src/mathOperations.js");
/* harmony import */ var _stat__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./stat */ "./node_modules/ml-matrix/src/stat.js");
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./util */ "./node_modules/ml-matrix/src/util.js");








class AbstractMatrix {
  static from1DArray(newRows, newColumns, newData) {
    let length = newRows * newColumns;
    if (length !== newData.length) {
      throw new RangeError('data length does not match given dimensions');
    }
    let newMatrix = new Matrix(newRows, newColumns);
    for (let row = 0; row < newRows; row++) {
      for (let column = 0; column < newColumns; column++) {
        newMatrix.set(row, column, newData[row * newColumns + column]);
      }
    }
    return newMatrix;
  }

  static rowVector(newData) {
    let vector = new Matrix(1, newData.length);
    for (let i = 0; i < newData.length; i++) {
      vector.set(0, i, newData[i]);
    }
    return vector;
  }

  static columnVector(newData) {
    let vector = new Matrix(newData.length, 1);
    for (let i = 0; i < newData.length; i++) {
      vector.set(i, 0, newData[i]);
    }
    return vector;
  }

  static zeros(rows, columns) {
    return new Matrix(rows, columns);
  }

  static ones(rows, columns) {
    return new Matrix(rows, columns).fill(1);
  }

  static rand(rows, columns, options = {}) {
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    const { random = Math.random } = options;
    let matrix = new Matrix(rows, columns);
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < columns; j++) {
        matrix.set(i, j, random());
      }
    }
    return matrix;
  }

  static randInt(rows, columns, options = {}) {
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    const { min = 0, max = 1000, random = Math.random } = options;
    if (!Number.isInteger(min)) throw new TypeError('min must be an integer');
    if (!Number.isInteger(max)) throw new TypeError('max must be an integer');
    if (min >= max) throw new RangeError('min must be smaller than max');
    let interval = max - min;
    let matrix = new Matrix(rows, columns);
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < columns; j++) {
        let value = min + Math.round(random() * interval);
        matrix.set(i, j, value);
      }
    }
    return matrix;
  }

  static eye(rows, columns, value) {
    if (columns === undefined) columns = rows;
    if (value === undefined) value = 1;
    let min = Math.min(rows, columns);
    let matrix = this.zeros(rows, columns);
    for (let i = 0; i < min; i++) {
      matrix.set(i, i, value);
    }
    return matrix;
  }

  static diag(data, rows, columns) {
    let l = data.length;
    if (rows === undefined) rows = l;
    if (columns === undefined) columns = rows;
    let min = Math.min(l, rows, columns);
    let matrix = this.zeros(rows, columns);
    for (let i = 0; i < min; i++) {
      matrix.set(i, i, data[i]);
    }
    return matrix;
  }

  static min(matrix1, matrix2) {
    matrix1 = this.checkMatrix(matrix1);
    matrix2 = this.checkMatrix(matrix2);
    let rows = matrix1.rows;
    let columns = matrix1.columns;
    let result = new Matrix(rows, columns);
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < columns; j++) {
        result.set(i, j, Math.min(matrix1.get(i, j), matrix2.get(i, j)));
      }
    }
    return result;
  }

  static max(matrix1, matrix2) {
    matrix1 = this.checkMatrix(matrix1);
    matrix2 = this.checkMatrix(matrix2);
    let rows = matrix1.rows;
    let columns = matrix1.columns;
    let result = new this(rows, columns);
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < columns; j++) {
        result.set(i, j, Math.max(matrix1.get(i, j), matrix2.get(i, j)));
      }
    }
    return result;
  }

  static checkMatrix(value) {
    return AbstractMatrix.isMatrix(value) ? value : new Matrix(value);
  }

  static isMatrix(value) {
    return value != null && value.klass === 'Matrix';
  }

  get size() {
    return this.rows * this.columns;
  }

  apply(callback) {
    if (typeof callback !== 'function') {
      throw new TypeError('callback must be a function');
    }
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        callback.call(this, i, j);
      }
    }
    return this;
  }

  to1DArray() {
    let array = [];
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        array.push(this.get(i, j));
      }
    }
    return array;
  }

  to2DArray() {
    let copy = [];
    for (let i = 0; i < this.rows; i++) {
      copy.push([]);
      for (let j = 0; j < this.columns; j++) {
        copy[i].push(this.get(i, j));
      }
    }
    return copy;
  }

  toJSON() {
    return this.to2DArray();
  }

  isRowVector() {
    return this.rows === 1;
  }

  isColumnVector() {
    return this.columns === 1;
  }

  isVector() {
    return this.rows === 1 || this.columns === 1;
  }

  isSquare() {
    return this.rows === this.columns;
  }

  isEmpty() {
    return this.rows === 0 || this.columns === 0;
  }

  isSymmetric() {
    if (this.isSquare()) {
      for (let i = 0; i < this.rows; i++) {
        for (let j = 0; j <= i; j++) {
          if (this.get(i, j) !== this.get(j, i)) {
            return false;
          }
        }
      }
      return true;
    }
    return false;
  }

  isEchelonForm() {
    let i = 0;
    let j = 0;
    let previousColumn = -1;
    let isEchelonForm = true;
    let checked = false;
    while (i < this.rows && isEchelonForm) {
      j = 0;
      checked = false;
      while (j < this.columns && checked === false) {
        if (this.get(i, j) === 0) {
          j++;
        } else if (this.get(i, j) === 1 && j > previousColumn) {
          checked = true;
          previousColumn = j;
        } else {
          isEchelonForm = false;
          checked = true;
        }
      }
      i++;
    }
    return isEchelonForm;
  }

  isReducedEchelonForm() {
    let i = 0;
    let j = 0;
    let previousColumn = -1;
    let isReducedEchelonForm = true;
    let checked = false;
    while (i < this.rows && isReducedEchelonForm) {
      j = 0;
      checked = false;
      while (j < this.columns && checked === false) {
        if (this.get(i, j) === 0) {
          j++;
        } else if (this.get(i, j) === 1 && j > previousColumn) {
          checked = true;
          previousColumn = j;
        } else {
          isReducedEchelonForm = false;
          checked = true;
        }
      }
      for (let k = j + 1; k < this.rows; k++) {
        if (this.get(i, k) !== 0) {
          isReducedEchelonForm = false;
        }
      }
      i++;
    }
    return isReducedEchelonForm;
  }

  echelonForm() {
    let result = this.clone();
    let h = 0;
    let k = 0;
    while (h < result.rows && k < result.columns) {
      let iMax = h;
      for (let i = h; i < result.rows; i++) {
        if (result.get(i, k) > result.get(iMax, k)) {
          iMax = i;
        }
      }
      if (result.get(iMax, k) === 0) {
        k++;
      } else {
        result.swapRows(h, iMax);
        let tmp = result.get(h, k);
        for (let j = k; j < result.columns; j++) {
          result.set(h, j, result.get(h, j) / tmp);
        }
        for (let i = h + 1; i < result.rows; i++) {
          let factor = result.get(i, k) / result.get(h, k);
          result.set(i, k, 0);
          for (let j = k + 1; j < result.columns; j++) {
            result.set(i, j, result.get(i, j) - result.get(h, j) * factor);
          }
        }
        h++;
        k++;
      }
    }
    return result;
  }

  reducedEchelonForm() {
    let result = this.echelonForm();
    let m = result.columns;
    let n = result.rows;
    let h = n - 1;
    while (h >= 0) {
      if (result.maxRow(h) === 0) {
        h--;
      } else {
        let p = 0;
        let pivot = false;
        while (p < n && pivot === false) {
          if (result.get(h, p) === 1) {
            pivot = true;
          } else {
            p++;
          }
        }
        for (let i = 0; i < h; i++) {
          let factor = result.get(i, p);
          for (let j = p; j < m; j++) {
            let tmp = result.get(i, j) - factor * result.get(h, j);
            result.set(i, j, tmp);
          }
        }
        h--;
      }
    }
    return result;
  }

  set() {
    throw new Error('set method is unimplemented');
  }

  get() {
    throw new Error('get method is unimplemented');
  }

  repeat(options = {}) {
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    const { rows = 1, columns = 1 } = options;
    if (!Number.isInteger(rows) || rows <= 0) {
      throw new TypeError('rows must be a positive integer');
    }
    if (!Number.isInteger(columns) || columns <= 0) {
      throw new TypeError('columns must be a positive integer');
    }
    let matrix = new Matrix(this.rows * rows, this.columns * columns);
    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < columns; j++) {
        matrix.setSubMatrix(this, this.rows * i, this.columns * j);
      }
    }
    return matrix;
  }

  fill(value) {
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, value);
      }
    }
    return this;
  }

  neg() {
    return this.mulS(-1);
  }

  getRow(index) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, index);
    let row = [];
    for (let i = 0; i < this.columns; i++) {
      row.push(this.get(index, i));
    }
    return row;
  }

  getRowVector(index) {
    return Matrix.rowVector(this.getRow(index));
  }

  setRow(index, array) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, index);
    array = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowVector)(this, array);
    for (let i = 0; i < this.columns; i++) {
      this.set(index, i, array[i]);
    }
    return this;
  }

  swapRows(row1, row2) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, row1);
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, row2);
    for (let i = 0; i < this.columns; i++) {
      let temp = this.get(row1, i);
      this.set(row1, i, this.get(row2, i));
      this.set(row2, i, temp);
    }
    return this;
  }

  getColumn(index) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, index);
    let column = [];
    for (let i = 0; i < this.rows; i++) {
      column.push(this.get(i, index));
    }
    return column;
  }

  getColumnVector(index) {
    return Matrix.columnVector(this.getColumn(index));
  }

  setColumn(index, array) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, index);
    array = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnVector)(this, array);
    for (let i = 0; i < this.rows; i++) {
      this.set(i, index, array[i]);
    }
    return this;
  }

  swapColumns(column1, column2) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, column1);
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, column2);
    for (let i = 0; i < this.rows; i++) {
      let temp = this.get(i, column1);
      this.set(i, column1, this.get(i, column2));
      this.set(i, column2, temp);
    }
    return this;
  }

  addRowVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) + vector[j]);
      }
    }
    return this;
  }

  subRowVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) - vector[j]);
      }
    }
    return this;
  }

  mulRowVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) * vector[j]);
      }
    }
    return this;
  }

  divRowVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) / vector[j]);
      }
    }
    return this;
  }

  addColumnVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) + vector[i]);
      }
    }
    return this;
  }

  subColumnVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) - vector[i]);
      }
    }
    return this;
  }

  mulColumnVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) * vector[i]);
      }
    }
    return this;
  }

  divColumnVector(vector) {
    vector = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnVector)(this, vector);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        this.set(i, j, this.get(i, j) / vector[i]);
      }
    }
    return this;
  }

  mulRow(index, value) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, index);
    for (let i = 0; i < this.columns; i++) {
      this.set(index, i, this.get(index, i) * value);
    }
    return this;
  }

  mulColumn(index, value) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, index);
    for (let i = 0; i < this.rows; i++) {
      this.set(i, index, this.get(i, index) * value);
    }
    return this;
  }

  max(by) {
    if (this.isEmpty()) {
      return NaN;
    }
    switch (by) {
      case 'row': {
        const max = new Array(this.rows).fill(Number.NEGATIVE_INFINITY);
        for (let row = 0; row < this.rows; row++) {
          for (let column = 0; column < this.columns; column++) {
            if (this.get(row, column) > max[row]) {
              max[row] = this.get(row, column);
            }
          }
        }
        return max;
      }
      case 'column': {
        const max = new Array(this.columns).fill(Number.NEGATIVE_INFINITY);
        for (let row = 0; row < this.rows; row++) {
          for (let column = 0; column < this.columns; column++) {
            if (this.get(row, column) > max[column]) {
              max[column] = this.get(row, column);
            }
          }
        }
        return max;
      }
      case undefined: {
        let max = this.get(0, 0);
        for (let row = 0; row < this.rows; row++) {
          for (let column = 0; column < this.columns; column++) {
            if (this.get(row, column) > max) {
              max = this.get(row, column);
            }
          }
        }
        return max;
      }
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  maxIndex() {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkNonEmpty)(this);
    let v = this.get(0, 0);
    let idx = [0, 0];
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        if (this.get(i, j) > v) {
          v = this.get(i, j);
          idx[0] = i;
          idx[1] = j;
        }
      }
    }
    return idx;
  }

  min(by) {
    if (this.isEmpty()) {
      return NaN;
    }

    switch (by) {
      case 'row': {
        const min = new Array(this.rows).fill(Number.POSITIVE_INFINITY);
        for (let row = 0; row < this.rows; row++) {
          for (let column = 0; column < this.columns; column++) {
            if (this.get(row, column) < min[row]) {
              min[row] = this.get(row, column);
            }
          }
        }
        return min;
      }
      case 'column': {
        const min = new Array(this.columns).fill(Number.POSITIVE_INFINITY);
        for (let row = 0; row < this.rows; row++) {
          for (let column = 0; column < this.columns; column++) {
            if (this.get(row, column) < min[column]) {
              min[column] = this.get(row, column);
            }
          }
        }
        return min;
      }
      case undefined: {
        let min = this.get(0, 0);
        for (let row = 0; row < this.rows; row++) {
          for (let column = 0; column < this.columns; column++) {
            if (this.get(row, column) < min) {
              min = this.get(row, column);
            }
          }
        }
        return min;
      }
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  minIndex() {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkNonEmpty)(this);
    let v = this.get(0, 0);
    let idx = [0, 0];
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        if (this.get(i, j) < v) {
          v = this.get(i, j);
          idx[0] = i;
          idx[1] = j;
        }
      }
    }
    return idx;
  }

  maxRow(row) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, row);
    if (this.isEmpty()) {
      return NaN;
    }
    let v = this.get(row, 0);
    for (let i = 1; i < this.columns; i++) {
      if (this.get(row, i) > v) {
        v = this.get(row, i);
      }
    }
    return v;
  }

  maxRowIndex(row) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, row);
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkNonEmpty)(this);
    let v = this.get(row, 0);
    let idx = [row, 0];
    for (let i = 1; i < this.columns; i++) {
      if (this.get(row, i) > v) {
        v = this.get(row, i);
        idx[1] = i;
      }
    }
    return idx;
  }

  minRow(row) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, row);
    if (this.isEmpty()) {
      return NaN;
    }
    let v = this.get(row, 0);
    for (let i = 1; i < this.columns; i++) {
      if (this.get(row, i) < v) {
        v = this.get(row, i);
      }
    }
    return v;
  }

  minRowIndex(row) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, row);
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkNonEmpty)(this);
    let v = this.get(row, 0);
    let idx = [row, 0];
    for (let i = 1; i < this.columns; i++) {
      if (this.get(row, i) < v) {
        v = this.get(row, i);
        idx[1] = i;
      }
    }
    return idx;
  }

  maxColumn(column) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, column);
    if (this.isEmpty()) {
      return NaN;
    }
    let v = this.get(0, column);
    for (let i = 1; i < this.rows; i++) {
      if (this.get(i, column) > v) {
        v = this.get(i, column);
      }
    }
    return v;
  }

  maxColumnIndex(column) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, column);
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkNonEmpty)(this);
    let v = this.get(0, column);
    let idx = [0, column];
    for (let i = 1; i < this.rows; i++) {
      if (this.get(i, column) > v) {
        v = this.get(i, column);
        idx[0] = i;
      }
    }
    return idx;
  }

  minColumn(column) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, column);
    if (this.isEmpty()) {
      return NaN;
    }
    let v = this.get(0, column);
    for (let i = 1; i < this.rows; i++) {
      if (this.get(i, column) < v) {
        v = this.get(i, column);
      }
    }
    return v;
  }

  minColumnIndex(column) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, column);
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkNonEmpty)(this);
    let v = this.get(0, column);
    let idx = [0, column];
    for (let i = 1; i < this.rows; i++) {
      if (this.get(i, column) < v) {
        v = this.get(i, column);
        idx[0] = i;
      }
    }
    return idx;
  }

  diag() {
    let min = Math.min(this.rows, this.columns);
    let diag = [];
    for (let i = 0; i < min; i++) {
      diag.push(this.get(i, i));
    }
    return diag;
  }

  norm(type = 'frobenius') {
    let result = 0;
    if (type === 'max') {
      return this.max();
    } else if (type === 'frobenius') {
      for (let i = 0; i < this.rows; i++) {
        for (let j = 0; j < this.columns; j++) {
          result = result + this.get(i, j) * this.get(i, j);
        }
      }
      return Math.sqrt(result);
    } else {
      throw new RangeError(`unknown norm type: ${type}`);
    }
  }

  cumulativeSum() {
    let sum = 0;
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        sum += this.get(i, j);
        this.set(i, j, sum);
      }
    }
    return this;
  }

  dot(vector2) {
    if (AbstractMatrix.isMatrix(vector2)) vector2 = vector2.to1DArray();
    let vector1 = this.to1DArray();
    if (vector1.length !== vector2.length) {
      throw new RangeError('vectors do not have the same size');
    }
    let dot = 0;
    for (let i = 0; i < vector1.length; i++) {
      dot += vector1[i] * vector2[i];
    }
    return dot;
  }

  mmul(other) {
    other = Matrix.checkMatrix(other);

    let m = this.rows;
    let n = this.columns;
    let p = other.columns;

    let result = new Matrix(m, p);

    let Bcolj = new Float64Array(n);
    for (let j = 0; j < p; j++) {
      for (let k = 0; k < n; k++) {
        Bcolj[k] = other.get(k, j);
      }

      for (let i = 0; i < m; i++) {
        let s = 0;
        for (let k = 0; k < n; k++) {
          s += this.get(i, k) * Bcolj[k];
        }

        result.set(i, j, s);
      }
    }
    return result;
  }

  strassen2x2(other) {
    other = Matrix.checkMatrix(other);
    let result = new Matrix(2, 2);
    const a11 = this.get(0, 0);
    const b11 = other.get(0, 0);
    const a12 = this.get(0, 1);
    const b12 = other.get(0, 1);
    const a21 = this.get(1, 0);
    const b21 = other.get(1, 0);
    const a22 = this.get(1, 1);
    const b22 = other.get(1, 1);

    // Compute intermediate values.
    const m1 = (a11 + a22) * (b11 + b22);
    const m2 = (a21 + a22) * b11;
    const m3 = a11 * (b12 - b22);
    const m4 = a22 * (b21 - b11);
    const m5 = (a11 + a12) * b22;
    const m6 = (a21 - a11) * (b11 + b12);
    const m7 = (a12 - a22) * (b21 + b22);

    // Combine intermediate values into the output.
    const c00 = m1 + m4 - m5 + m7;
    const c01 = m3 + m5;
    const c10 = m2 + m4;
    const c11 = m1 - m2 + m3 + m6;

    result.set(0, 0, c00);
    result.set(0, 1, c01);
    result.set(1, 0, c10);
    result.set(1, 1, c11);
    return result;
  }

  strassen3x3(other) {
    other = Matrix.checkMatrix(other);
    let result = new Matrix(3, 3);

    const a00 = this.get(0, 0);
    const a01 = this.get(0, 1);
    const a02 = this.get(0, 2);
    const a10 = this.get(1, 0);
    const a11 = this.get(1, 1);
    const a12 = this.get(1, 2);
    const a20 = this.get(2, 0);
    const a21 = this.get(2, 1);
    const a22 = this.get(2, 2);

    const b00 = other.get(0, 0);
    const b01 = other.get(0, 1);
    const b02 = other.get(0, 2);
    const b10 = other.get(1, 0);
    const b11 = other.get(1, 1);
    const b12 = other.get(1, 2);
    const b20 = other.get(2, 0);
    const b21 = other.get(2, 1);
    const b22 = other.get(2, 2);

    const m1 = (a00 + a01 + a02 - a10 - a11 - a21 - a22) * b11;
    const m2 = (a00 - a10) * (-b01 + b11);
    const m3 = a11 * (-b00 + b01 + b10 - b11 - b12 - b20 + b22);
    const m4 = (-a00 + a10 + a11) * (b00 - b01 + b11);
    const m5 = (a10 + a11) * (-b00 + b01);
    const m6 = a00 * b00;
    const m7 = (-a00 + a20 + a21) * (b00 - b02 + b12);
    const m8 = (-a00 + a20) * (b02 - b12);
    const m9 = (a20 + a21) * (-b00 + b02);
    const m10 = (a00 + a01 + a02 - a11 - a12 - a20 - a21) * b12;
    const m11 = a21 * (-b00 + b02 + b10 - b11 - b12 - b20 + b21);
    const m12 = (-a02 + a21 + a22) * (b11 + b20 - b21);
    const m13 = (a02 - a22) * (b11 - b21);
    const m14 = a02 * b20;
    const m15 = (a21 + a22) * (-b20 + b21);
    const m16 = (-a02 + a11 + a12) * (b12 + b20 - b22);
    const m17 = (a02 - a12) * (b12 - b22);
    const m18 = (a11 + a12) * (-b20 + b22);
    const m19 = a01 * b10;
    const m20 = a12 * b21;
    const m21 = a10 * b02;
    const m22 = a20 * b01;
    const m23 = a22 * b22;

    const c00 = m6 + m14 + m19;
    const c01 = m1 + m4 + m5 + m6 + m12 + m14 + m15;
    const c02 = m6 + m7 + m9 + m10 + m14 + m16 + m18;
    const c10 = m2 + m3 + m4 + m6 + m14 + m16 + m17;
    const c11 = m2 + m4 + m5 + m6 + m20;
    const c12 = m14 + m16 + m17 + m18 + m21;
    const c20 = m6 + m7 + m8 + m11 + m12 + m13 + m14;
    const c21 = m12 + m13 + m14 + m15 + m22;
    const c22 = m6 + m7 + m8 + m9 + m23;

    result.set(0, 0, c00);
    result.set(0, 1, c01);
    result.set(0, 2, c02);
    result.set(1, 0, c10);
    result.set(1, 1, c11);
    result.set(1, 2, c12);
    result.set(2, 0, c20);
    result.set(2, 1, c21);
    result.set(2, 2, c22);
    return result;
  }

  mmulStrassen(y) {
    y = Matrix.checkMatrix(y);
    let x = this.clone();
    let r1 = x.rows;
    let c1 = x.columns;
    let r2 = y.rows;
    let c2 = y.columns;
    if (c1 !== r2) {
      // eslint-disable-next-line no-console
      console.warn(
        `Multiplying ${r1} x ${c1} and ${r2} x ${c2} matrix: dimensions do not match.`,
      );
    }

    // Put a matrix into the top left of a matrix of zeros.
    // `rows` and `cols` are the dimensions of the output matrix.
    function embed(mat, rows, cols) {
      let r = mat.rows;
      let c = mat.columns;
      if (r === rows && c === cols) {
        return mat;
      } else {
        let resultat = AbstractMatrix.zeros(rows, cols);
        resultat = resultat.setSubMatrix(mat, 0, 0);
        return resultat;
      }
    }

    // Make sure both matrices are the same size.
    // This is exclusively for simplicity:
    // this algorithm can be implemented with matrices of different sizes.

    let r = Math.max(r1, r2);
    let c = Math.max(c1, c2);
    x = embed(x, r, c);
    y = embed(y, r, c);

    // Our recursive multiplication function.
    function blockMult(a, b, rows, cols) {
      // For small matrices, resort to naive multiplication.
      if (rows <= 512 || cols <= 512) {
        return a.mmul(b); // a is equivalent to this
      }

      // Apply dynamic padding.
      if (rows % 2 === 1 && cols % 2 === 1) {
        a = embed(a, rows + 1, cols + 1);
        b = embed(b, rows + 1, cols + 1);
      } else if (rows % 2 === 1) {
        a = embed(a, rows + 1, cols);
        b = embed(b, rows + 1, cols);
      } else if (cols % 2 === 1) {
        a = embed(a, rows, cols + 1);
        b = embed(b, rows, cols + 1);
      }

      let halfRows = parseInt(a.rows / 2, 10);
      let halfCols = parseInt(a.columns / 2, 10);
      // Subdivide input matrices.
      let a11 = a.subMatrix(0, halfRows - 1, 0, halfCols - 1);
      let b11 = b.subMatrix(0, halfRows - 1, 0, halfCols - 1);

      let a12 = a.subMatrix(0, halfRows - 1, halfCols, a.columns - 1);
      let b12 = b.subMatrix(0, halfRows - 1, halfCols, b.columns - 1);

      let a21 = a.subMatrix(halfRows, a.rows - 1, 0, halfCols - 1);
      let b21 = b.subMatrix(halfRows, b.rows - 1, 0, halfCols - 1);

      let a22 = a.subMatrix(halfRows, a.rows - 1, halfCols, a.columns - 1);
      let b22 = b.subMatrix(halfRows, b.rows - 1, halfCols, b.columns - 1);

      // Compute intermediate values.
      let m1 = blockMult(
        AbstractMatrix.add(a11, a22),
        AbstractMatrix.add(b11, b22),
        halfRows,
        halfCols,
      );
      let m2 = blockMult(AbstractMatrix.add(a21, a22), b11, halfRows, halfCols);
      let m3 = blockMult(a11, AbstractMatrix.sub(b12, b22), halfRows, halfCols);
      let m4 = blockMult(a22, AbstractMatrix.sub(b21, b11), halfRows, halfCols);
      let m5 = blockMult(AbstractMatrix.add(a11, a12), b22, halfRows, halfCols);
      let m6 = blockMult(
        AbstractMatrix.sub(a21, a11),
        AbstractMatrix.add(b11, b12),
        halfRows,
        halfCols,
      );
      let m7 = blockMult(
        AbstractMatrix.sub(a12, a22),
        AbstractMatrix.add(b21, b22),
        halfRows,
        halfCols,
      );

      // Combine intermediate values into the output.
      let c11 = AbstractMatrix.add(m1, m4);
      c11.sub(m5);
      c11.add(m7);
      let c12 = AbstractMatrix.add(m3, m5);
      let c21 = AbstractMatrix.add(m2, m4);
      let c22 = AbstractMatrix.sub(m1, m2);
      c22.add(m3);
      c22.add(m6);

      // Crop output to the desired size (undo dynamic padding).
      let resultat = AbstractMatrix.zeros(2 * c11.rows, 2 * c11.columns);
      resultat = resultat.setSubMatrix(c11, 0, 0);
      resultat = resultat.setSubMatrix(c12, c11.rows, 0);
      resultat = resultat.setSubMatrix(c21, 0, c11.columns);
      resultat = resultat.setSubMatrix(c22, c11.rows, c11.columns);
      return resultat.subMatrix(0, rows - 1, 0, cols - 1);
    }

    return blockMult(x, y, r, c);
  }

  scaleRows(options = {}) {
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    const { min = 0, max = 1 } = options;
    if (!Number.isFinite(min)) throw new TypeError('min must be a number');
    if (!Number.isFinite(max)) throw new TypeError('max must be a number');
    if (min >= max) throw new RangeError('min must be smaller than max');
    let newMatrix = new Matrix(this.rows, this.columns);
    for (let i = 0; i < this.rows; i++) {
      const row = this.getRow(i);
      if (row.length > 0) {
        (0,ml_array_rescale__WEBPACK_IMPORTED_MODULE_1__["default"])(row, { min, max, output: row });
      }
      newMatrix.setRow(i, row);
    }
    return newMatrix;
  }

  scaleColumns(options = {}) {
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    const { min = 0, max = 1 } = options;
    if (!Number.isFinite(min)) throw new TypeError('min must be a number');
    if (!Number.isFinite(max)) throw new TypeError('max must be a number');
    if (min >= max) throw new RangeError('min must be smaller than max');
    let newMatrix = new Matrix(this.rows, this.columns);
    for (let i = 0; i < this.columns; i++) {
      const column = this.getColumn(i);
      if (column.length) {
        (0,ml_array_rescale__WEBPACK_IMPORTED_MODULE_1__["default"])(column, {
          min: min,
          max: max,
          output: column,
        });
      }
      newMatrix.setColumn(i, column);
    }
    return newMatrix;
  }

  flipRows() {
    const middle = Math.ceil(this.columns / 2);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < middle; j++) {
        let first = this.get(i, j);
        let last = this.get(i, this.columns - 1 - j);
        this.set(i, j, last);
        this.set(i, this.columns - 1 - j, first);
      }
    }
    return this;
  }

  flipColumns() {
    const middle = Math.ceil(this.rows / 2);
    for (let j = 0; j < this.columns; j++) {
      for (let i = 0; i < middle; i++) {
        let first = this.get(i, j);
        let last = this.get(this.rows - 1 - i, j);
        this.set(i, j, last);
        this.set(this.rows - 1 - i, j, first);
      }
    }
    return this;
  }

  kroneckerProduct(other) {
    other = Matrix.checkMatrix(other);

    let m = this.rows;
    let n = this.columns;
    let p = other.rows;
    let q = other.columns;

    let result = new Matrix(m * p, n * q);
    for (let i = 0; i < m; i++) {
      for (let j = 0; j < n; j++) {
        for (let k = 0; k < p; k++) {
          for (let l = 0; l < q; l++) {
            result.set(p * i + k, q * j + l, this.get(i, j) * other.get(k, l));
          }
        }
      }
    }
    return result;
  }

  kroneckerSum(other) {
    other = Matrix.checkMatrix(other);
    if (!this.isSquare() || !other.isSquare()) {
      throw new Error('Kronecker Sum needs two Square Matrices');
    }
    let m = this.rows;
    let n = other.rows;
    let AxI = this.kroneckerProduct(Matrix.eye(n, n));
    let IxB = Matrix.eye(m, m).kroneckerProduct(other);
    return AxI.add(IxB);
  }

  transpose() {
    let result = new Matrix(this.columns, this.rows);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.columns; j++) {
        result.set(j, i, this.get(i, j));
      }
    }
    return result;
  }

  sortRows(compareFunction = compareNumbers) {
    for (let i = 0; i < this.rows; i++) {
      this.setRow(i, this.getRow(i).sort(compareFunction));
    }
    return this;
  }

  sortColumns(compareFunction = compareNumbers) {
    for (let i = 0; i < this.columns; i++) {
      this.setColumn(i, this.getColumn(i).sort(compareFunction));
    }
    return this;
  }

  subMatrix(startRow, endRow, startColumn, endColumn) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRange)(this, startRow, endRow, startColumn, endColumn);
    let newMatrix = new Matrix(
      endRow - startRow + 1,
      endColumn - startColumn + 1,
    );
    for (let i = startRow; i <= endRow; i++) {
      for (let j = startColumn; j <= endColumn; j++) {
        newMatrix.set(i - startRow, j - startColumn, this.get(i, j));
      }
    }
    return newMatrix;
  }

  subMatrixRow(indices, startColumn, endColumn) {
    if (startColumn === undefined) startColumn = 0;
    if (endColumn === undefined) endColumn = this.columns - 1;
    if (
      startColumn > endColumn ||
      startColumn < 0 ||
      startColumn >= this.columns ||
      endColumn < 0 ||
      endColumn >= this.columns
    ) {
      throw new RangeError('Argument out of range');
    }

    let newMatrix = new Matrix(indices.length, endColumn - startColumn + 1);
    for (let i = 0; i < indices.length; i++) {
      for (let j = startColumn; j <= endColumn; j++) {
        if (indices[i] < 0 || indices[i] >= this.rows) {
          throw new RangeError(`Row index out of range: ${indices[i]}`);
        }
        newMatrix.set(i, j - startColumn, this.get(indices[i], j));
      }
    }
    return newMatrix;
  }

  subMatrixColumn(indices, startRow, endRow) {
    if (startRow === undefined) startRow = 0;
    if (endRow === undefined) endRow = this.rows - 1;
    if (
      startRow > endRow ||
      startRow < 0 ||
      startRow >= this.rows ||
      endRow < 0 ||
      endRow >= this.rows
    ) {
      throw new RangeError('Argument out of range');
    }

    let newMatrix = new Matrix(endRow - startRow + 1, indices.length);
    for (let i = 0; i < indices.length; i++) {
      for (let j = startRow; j <= endRow; j++) {
        if (indices[i] < 0 || indices[i] >= this.columns) {
          throw new RangeError(`Column index out of range: ${indices[i]}`);
        }
        newMatrix.set(j - startRow, i, this.get(j, indices[i]));
      }
    }
    return newMatrix;
  }

  setSubMatrix(matrix, startRow, startColumn) {
    matrix = Matrix.checkMatrix(matrix);
    if (matrix.isEmpty()) {
      return this;
    }
    let endRow = startRow + matrix.rows - 1;
    let endColumn = startColumn + matrix.columns - 1;
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRange)(this, startRow, endRow, startColumn, endColumn);
    for (let i = 0; i < matrix.rows; i++) {
      for (let j = 0; j < matrix.columns; j++) {
        this.set(startRow + i, startColumn + j, matrix.get(i, j));
      }
    }
    return this;
  }

  selection(rowIndices, columnIndices) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndices)(this, rowIndices);
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndices)(this, columnIndices);
    let newMatrix = new Matrix(rowIndices.length, columnIndices.length);
    for (let i = 0; i < rowIndices.length; i++) {
      let rowIndex = rowIndices[i];
      for (let j = 0; j < columnIndices.length; j++) {
        let columnIndex = columnIndices[j];
        newMatrix.set(i, j, this.get(rowIndex, columnIndex));
      }
    }
    return newMatrix;
  }

  trace() {
    let min = Math.min(this.rows, this.columns);
    let trace = 0;
    for (let i = 0; i < min; i++) {
      trace += this.get(i, i);
    }
    return trace;
  }

  clone() {
    let newMatrix = new Matrix(this.rows, this.columns);
    for (let row = 0; row < this.rows; row++) {
      for (let column = 0; column < this.columns; column++) {
        newMatrix.set(row, column, this.get(row, column));
      }
    }
    return newMatrix;
  }

  sum(by) {
    switch (by) {
      case 'row':
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.sumByRow)(this);
      case 'column':
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.sumByColumn)(this);
      case undefined:
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.sumAll)(this);
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  product(by) {
    switch (by) {
      case 'row':
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.productByRow)(this);
      case 'column':
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.productByColumn)(this);
      case undefined:
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.productAll)(this);
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  mean(by) {
    const sum = this.sum(by);
    switch (by) {
      case 'row': {
        for (let i = 0; i < this.rows; i++) {
          sum[i] /= this.columns;
        }
        return sum;
      }
      case 'column': {
        for (let i = 0; i < this.columns; i++) {
          sum[i] /= this.rows;
        }
        return sum;
      }
      case undefined:
        return sum / this.size;
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  variance(by, options = {}) {
    if (typeof by === 'object') {
      options = by;
      by = undefined;
    }
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    const { unbiased = true, mean = this.mean(by) } = options;
    if (typeof unbiased !== 'boolean') {
      throw new TypeError('unbiased must be a boolean');
    }
    switch (by) {
      case 'row': {
        if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(mean)) {
          throw new TypeError('mean must be an array');
        }
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.varianceByRow)(this, unbiased, mean);
      }
      case 'column': {
        if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(mean)) {
          throw new TypeError('mean must be an array');
        }
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.varianceByColumn)(this, unbiased, mean);
      }
      case undefined: {
        if (typeof mean !== 'number') {
          throw new TypeError('mean must be a number');
        }
        return (0,_stat__WEBPACK_IMPORTED_MODULE_3__.varianceAll)(this, unbiased, mean);
      }
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  standardDeviation(by, options) {
    if (typeof by === 'object') {
      options = by;
      by = undefined;
    }
    const variance = this.variance(by, options);
    if (by === undefined) {
      return Math.sqrt(variance);
    } else {
      for (let i = 0; i < variance.length; i++) {
        variance[i] = Math.sqrt(variance[i]);
      }
      return variance;
    }
  }

  center(by, options = {}) {
    if (typeof by === 'object') {
      options = by;
      by = undefined;
    }
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    const { center = this.mean(by) } = options;
    switch (by) {
      case 'row': {
        if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(center)) {
          throw new TypeError('center must be an array');
        }
        (0,_stat__WEBPACK_IMPORTED_MODULE_3__.centerByRow)(this, center);
        return this;
      }
      case 'column': {
        if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(center)) {
          throw new TypeError('center must be an array');
        }
        (0,_stat__WEBPACK_IMPORTED_MODULE_3__.centerByColumn)(this, center);
        return this;
      }
      case undefined: {
        if (typeof center !== 'number') {
          throw new TypeError('center must be a number');
        }
        (0,_stat__WEBPACK_IMPORTED_MODULE_3__.centerAll)(this, center);
        return this;
      }
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  scale(by, options = {}) {
    if (typeof by === 'object') {
      options = by;
      by = undefined;
    }
    if (typeof options !== 'object') {
      throw new TypeError('options must be an object');
    }
    let scale = options.scale;
    switch (by) {
      case 'row': {
        if (scale === undefined) {
          scale = (0,_stat__WEBPACK_IMPORTED_MODULE_3__.getScaleByRow)(this);
        } else if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(scale)) {
          throw new TypeError('scale must be an array');
        }
        (0,_stat__WEBPACK_IMPORTED_MODULE_3__.scaleByRow)(this, scale);
        return this;
      }
      case 'column': {
        if (scale === undefined) {
          scale = (0,_stat__WEBPACK_IMPORTED_MODULE_3__.getScaleByColumn)(this);
        } else if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(scale)) {
          throw new TypeError('scale must be an array');
        }
        (0,_stat__WEBPACK_IMPORTED_MODULE_3__.scaleByColumn)(this, scale);
        return this;
      }
      case undefined: {
        if (scale === undefined) {
          scale = (0,_stat__WEBPACK_IMPORTED_MODULE_3__.getScaleAll)(this);
        } else if (typeof scale !== 'number') {
          throw new TypeError('scale must be a number');
        }
        (0,_stat__WEBPACK_IMPORTED_MODULE_3__.scaleAll)(this, scale);
        return this;
      }
      default:
        throw new Error(`invalid option: ${by}`);
    }
  }

  toString(options) {
    return (0,_inspect__WEBPACK_IMPORTED_MODULE_4__.inspectMatrixWithOptions)(this, options);
  }
}

AbstractMatrix.prototype.klass = 'Matrix';
if (typeof Symbol !== 'undefined') {
  AbstractMatrix.prototype[Symbol.for('nodejs.util.inspect.custom')] =
    _inspect__WEBPACK_IMPORTED_MODULE_4__.inspectMatrix;
}

function compareNumbers(a, b) {
  return a - b;
}

function isArrayOfNumbers(array) {
  return array.every((element) => {
    return typeof element === 'number';
  });
}

// Synonyms
AbstractMatrix.random = AbstractMatrix.rand;
AbstractMatrix.randomInt = AbstractMatrix.randInt;
AbstractMatrix.diagonal = AbstractMatrix.diag;
AbstractMatrix.prototype.diagonal = AbstractMatrix.prototype.diag;
AbstractMatrix.identity = AbstractMatrix.eye;
AbstractMatrix.prototype.negate = AbstractMatrix.prototype.neg;
AbstractMatrix.prototype.tensorProduct =
  AbstractMatrix.prototype.kroneckerProduct;

class Matrix extends AbstractMatrix {
  constructor(nRows, nColumns) {
    super();
    if (Matrix.isMatrix(nRows)) {
      // eslint-disable-next-line no-constructor-return
      return nRows.clone();
    } else if (Number.isInteger(nRows) && nRows >= 0) {
      // Create an empty matrix
      this.data = [];
      if (Number.isInteger(nColumns) && nColumns >= 0) {
        for (let i = 0; i < nRows; i++) {
          this.data.push(new Float64Array(nColumns));
        }
      } else {
        throw new TypeError('nColumns must be a positive integer');
      }
    } else if ((0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(nRows)) {
      // Copy the values from the 2D array
      const arrayData = nRows;
      nRows = arrayData.length;
      nColumns = nRows ? arrayData[0].length : 0;
      if (typeof nColumns !== 'number') {
        throw new TypeError(
          'Data must be a 2D array with at least one element',
        );
      }
      this.data = [];
      for (let i = 0; i < nRows; i++) {
        if (arrayData[i].length !== nColumns) {
          throw new RangeError('Inconsistent array dimensions');
        }
        if (!isArrayOfNumbers(arrayData[i])) {
          throw new TypeError('Input data contains non-numeric values');
        }
        this.data.push(Float64Array.from(arrayData[i]));
      }
    } else {
      throw new TypeError(
        'First argument must be a positive number or an array',
      );
    }
    this.rows = nRows;
    this.columns = nColumns;
  }

  set(rowIndex, columnIndex, value) {
    this.data[rowIndex][columnIndex] = value;
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.data[rowIndex][columnIndex];
  }

  removeRow(index) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, index);
    this.data.splice(index, 1);
    this.rows -= 1;
    return this;
  }

  addRow(index, array) {
    if (array === undefined) {
      array = index;
      index = this.rows;
    }
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowIndex)(this, index, true);
    array = Float64Array.from((0,_util__WEBPACK_IMPORTED_MODULE_2__.checkRowVector)(this, array));
    this.data.splice(index, 0, array);
    this.rows += 1;
    return this;
  }

  removeColumn(index) {
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, index);
    for (let i = 0; i < this.rows; i++) {
      const newRow = new Float64Array(this.columns - 1);
      for (let j = 0; j < index; j++) {
        newRow[j] = this.data[i][j];
      }
      for (let j = index + 1; j < this.columns; j++) {
        newRow[j - 1] = this.data[i][j];
      }
      this.data[i] = newRow;
    }
    this.columns -= 1;
    return this;
  }

  addColumn(index, array) {
    if (typeof array === 'undefined') {
      array = index;
      index = this.columns;
    }
    (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnIndex)(this, index, true);
    array = (0,_util__WEBPACK_IMPORTED_MODULE_2__.checkColumnVector)(this, array);
    for (let i = 0; i < this.rows; i++) {
      const newRow = new Float64Array(this.columns + 1);
      let j = 0;
      for (; j < index; j++) {
        newRow[j] = this.data[i][j];
      }
      newRow[j++] = array[i];
      for (; j < this.columns + 1; j++) {
        newRow[j] = this.data[i][j - 1];
      }
      this.data[i] = newRow;
    }
    this.columns += 1;
    return this;
  }
}

(0,_mathOperations__WEBPACK_IMPORTED_MODULE_5__.installMathOperations)(AbstractMatrix, Matrix);


/***/ }),

/***/ "./node_modules/ml-matrix/src/pseudoInverse.js":
/*!*****************************************************!*\
  !*** ./node_modules/ml-matrix/src/pseudoInverse.js ***!
  \*****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "pseudoInverse": () => (/* binding */ pseudoInverse)
/* harmony export */ });
/* harmony import */ var _dc_svd__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./dc/svd */ "./node_modules/ml-matrix/src/dc/svd.js");
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./matrix */ "./node_modules/ml-matrix/src/matrix.js");



function pseudoInverse(matrix, threshold = Number.EPSILON) {
  matrix = _matrix__WEBPACK_IMPORTED_MODULE_0__["default"].checkMatrix(matrix);
  if (matrix.isEmpty()) {
    // with a zero dimension, the pseudo-inverse is the transpose, since all 0xn and nx0 matrices are singular
    // (0xn)*(nx0)*(0xn) = 0xn
    // (nx0)*(0xn)*(nx0) = nx0
    return matrix.transpose();
  }
  let svdSolution = new _dc_svd__WEBPACK_IMPORTED_MODULE_1__["default"](matrix, { autoTranspose: true });

  let U = svdSolution.leftSingularVectors;
  let V = svdSolution.rightSingularVectors;
  let s = svdSolution.diagonal;

  for (let i = 0; i < s.length; i++) {
    if (Math.abs(s[i]) > threshold) {
      s[i] = 1.0 / s[i];
    } else {
      s[i] = 0.0;
    }
  }

  return V.mmul(_matrix__WEBPACK_IMPORTED_MODULE_0__["default"].diag(s).mmul(U.transpose()));
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/stat.js":
/*!********************************************!*\
  !*** ./node_modules/ml-matrix/src/stat.js ***!
  \********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "centerAll": () => (/* binding */ centerAll),
/* harmony export */   "centerByColumn": () => (/* binding */ centerByColumn),
/* harmony export */   "centerByRow": () => (/* binding */ centerByRow),
/* harmony export */   "getScaleAll": () => (/* binding */ getScaleAll),
/* harmony export */   "getScaleByColumn": () => (/* binding */ getScaleByColumn),
/* harmony export */   "getScaleByRow": () => (/* binding */ getScaleByRow),
/* harmony export */   "productAll": () => (/* binding */ productAll),
/* harmony export */   "productByColumn": () => (/* binding */ productByColumn),
/* harmony export */   "productByRow": () => (/* binding */ productByRow),
/* harmony export */   "scaleAll": () => (/* binding */ scaleAll),
/* harmony export */   "scaleByColumn": () => (/* binding */ scaleByColumn),
/* harmony export */   "scaleByRow": () => (/* binding */ scaleByRow),
/* harmony export */   "sumAll": () => (/* binding */ sumAll),
/* harmony export */   "sumByColumn": () => (/* binding */ sumByColumn),
/* harmony export */   "sumByRow": () => (/* binding */ sumByRow),
/* harmony export */   "varianceAll": () => (/* binding */ varianceAll),
/* harmony export */   "varianceByColumn": () => (/* binding */ varianceByColumn),
/* harmony export */   "varianceByRow": () => (/* binding */ varianceByRow)
/* harmony export */ });
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./util */ "./node_modules/ml-matrix/src/util.js");


function sumByRow(matrix) {
  let sum = (0,_util__WEBPACK_IMPORTED_MODULE_0__.newArray)(matrix.rows);
  for (let i = 0; i < matrix.rows; ++i) {
    for (let j = 0; j < matrix.columns; ++j) {
      sum[i] += matrix.get(i, j);
    }
  }
  return sum;
}

function sumByColumn(matrix) {
  let sum = (0,_util__WEBPACK_IMPORTED_MODULE_0__.newArray)(matrix.columns);
  for (let i = 0; i < matrix.rows; ++i) {
    for (let j = 0; j < matrix.columns; ++j) {
      sum[j] += matrix.get(i, j);
    }
  }
  return sum;
}

function sumAll(matrix) {
  let v = 0;
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      v += matrix.get(i, j);
    }
  }
  return v;
}

function productByRow(matrix) {
  let sum = (0,_util__WEBPACK_IMPORTED_MODULE_0__.newArray)(matrix.rows, 1);
  for (let i = 0; i < matrix.rows; ++i) {
    for (let j = 0; j < matrix.columns; ++j) {
      sum[i] *= matrix.get(i, j);
    }
  }
  return sum;
}

function productByColumn(matrix) {
  let sum = (0,_util__WEBPACK_IMPORTED_MODULE_0__.newArray)(matrix.columns, 1);
  for (let i = 0; i < matrix.rows; ++i) {
    for (let j = 0; j < matrix.columns; ++j) {
      sum[j] *= matrix.get(i, j);
    }
  }
  return sum;
}

function productAll(matrix) {
  let v = 1;
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      v *= matrix.get(i, j);
    }
  }
  return v;
}

function varianceByRow(matrix, unbiased, mean) {
  const rows = matrix.rows;
  const cols = matrix.columns;
  const variance = [];

  for (let i = 0; i < rows; i++) {
    let sum1 = 0;
    let sum2 = 0;
    let x = 0;
    for (let j = 0; j < cols; j++) {
      x = matrix.get(i, j) - mean[i];
      sum1 += x;
      sum2 += x * x;
    }
    if (unbiased) {
      variance.push((sum2 - (sum1 * sum1) / cols) / (cols - 1));
    } else {
      variance.push((sum2 - (sum1 * sum1) / cols) / cols);
    }
  }
  return variance;
}

function varianceByColumn(matrix, unbiased, mean) {
  const rows = matrix.rows;
  const cols = matrix.columns;
  const variance = [];

  for (let j = 0; j < cols; j++) {
    let sum1 = 0;
    let sum2 = 0;
    let x = 0;
    for (let i = 0; i < rows; i++) {
      x = matrix.get(i, j) - mean[j];
      sum1 += x;
      sum2 += x * x;
    }
    if (unbiased) {
      variance.push((sum2 - (sum1 * sum1) / rows) / (rows - 1));
    } else {
      variance.push((sum2 - (sum1 * sum1) / rows) / rows);
    }
  }
  return variance;
}

function varianceAll(matrix, unbiased, mean) {
  const rows = matrix.rows;
  const cols = matrix.columns;
  const size = rows * cols;

  let sum1 = 0;
  let sum2 = 0;
  let x = 0;
  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) {
      x = matrix.get(i, j) - mean;
      sum1 += x;
      sum2 += x * x;
    }
  }
  if (unbiased) {
    return (sum2 - (sum1 * sum1) / size) / (size - 1);
  } else {
    return (sum2 - (sum1 * sum1) / size) / size;
  }
}

function centerByRow(matrix, mean) {
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      matrix.set(i, j, matrix.get(i, j) - mean[i]);
    }
  }
}

function centerByColumn(matrix, mean) {
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      matrix.set(i, j, matrix.get(i, j) - mean[j]);
    }
  }
}

function centerAll(matrix, mean) {
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      matrix.set(i, j, matrix.get(i, j) - mean);
    }
  }
}

function getScaleByRow(matrix) {
  const scale = [];
  for (let i = 0; i < matrix.rows; i++) {
    let sum = 0;
    for (let j = 0; j < matrix.columns; j++) {
      sum += Math.pow(matrix.get(i, j), 2) / (matrix.columns - 1);
    }
    scale.push(Math.sqrt(sum));
  }
  return scale;
}

function scaleByRow(matrix, scale) {
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      matrix.set(i, j, matrix.get(i, j) / scale[i]);
    }
  }
}

function getScaleByColumn(matrix) {
  const scale = [];
  for (let j = 0; j < matrix.columns; j++) {
    let sum = 0;
    for (let i = 0; i < matrix.rows; i++) {
      sum += Math.pow(matrix.get(i, j), 2) / (matrix.rows - 1);
    }
    scale.push(Math.sqrt(sum));
  }
  return scale;
}

function scaleByColumn(matrix, scale) {
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      matrix.set(i, j, matrix.get(i, j) / scale[j]);
    }
  }
}

function getScaleAll(matrix) {
  const divider = matrix.size - 1;
  let sum = 0;
  for (let j = 0; j < matrix.columns; j++) {
    for (let i = 0; i < matrix.rows; i++) {
      sum += Math.pow(matrix.get(i, j), 2) / divider;
    }
  }
  return Math.sqrt(sum);
}

function scaleAll(matrix, scale) {
  for (let i = 0; i < matrix.rows; i++) {
    for (let j = 0; j < matrix.columns; j++) {
      matrix.set(i, j, matrix.get(i, j) / scale);
    }
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/util.js":
/*!********************************************!*\
  !*** ./node_modules/ml-matrix/src/util.js ***!
  \********************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "checkColumnIndex": () => (/* binding */ checkColumnIndex),
/* harmony export */   "checkColumnIndices": () => (/* binding */ checkColumnIndices),
/* harmony export */   "checkColumnVector": () => (/* binding */ checkColumnVector),
/* harmony export */   "checkNonEmpty": () => (/* binding */ checkNonEmpty),
/* harmony export */   "checkRange": () => (/* binding */ checkRange),
/* harmony export */   "checkRowIndex": () => (/* binding */ checkRowIndex),
/* harmony export */   "checkRowIndices": () => (/* binding */ checkRowIndices),
/* harmony export */   "checkRowVector": () => (/* binding */ checkRowVector),
/* harmony export */   "newArray": () => (/* binding */ newArray)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");


/**
 * @private
 * Check that a row index is not out of bounds
 * @param {Matrix} matrix
 * @param {number} index
 * @param {boolean} [outer]
 */
function checkRowIndex(matrix, index, outer) {
  let max = outer ? matrix.rows : matrix.rows - 1;
  if (index < 0 || index > max) {
    throw new RangeError('Row index out of range');
  }
}

/**
 * @private
 * Check that a column index is not out of bounds
 * @param {Matrix} matrix
 * @param {number} index
 * @param {boolean} [outer]
 */
function checkColumnIndex(matrix, index, outer) {
  let max = outer ? matrix.columns : matrix.columns - 1;
  if (index < 0 || index > max) {
    throw new RangeError('Column index out of range');
  }
}

/**
 * @private
 * Check that the provided vector is an array with the right length
 * @param {Matrix} matrix
 * @param {Array|Matrix} vector
 * @return {Array}
 * @throws {RangeError}
 */
function checkRowVector(matrix, vector) {
  if (vector.to1DArray) {
    vector = vector.to1DArray();
  }
  if (vector.length !== matrix.columns) {
    throw new RangeError(
      'vector size must be the same as the number of columns',
    );
  }
  return vector;
}

/**
 * @private
 * Check that the provided vector is an array with the right length
 * @param {Matrix} matrix
 * @param {Array|Matrix} vector
 * @return {Array}
 * @throws {RangeError}
 */
function checkColumnVector(matrix, vector) {
  if (vector.to1DArray) {
    vector = vector.to1DArray();
  }
  if (vector.length !== matrix.rows) {
    throw new RangeError('vector size must be the same as the number of rows');
  }
  return vector;
}

function checkRowIndices(matrix, rowIndices) {
  if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(rowIndices)) {
    throw new TypeError('row indices must be an array');
  }

  for (let i = 0; i < rowIndices.length; i++) {
    if (rowIndices[i] < 0 || rowIndices[i] >= matrix.rows) {
      throw new RangeError('row indices are out of range');
    }
  }
}

function checkColumnIndices(matrix, columnIndices) {
  if (!(0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(columnIndices)) {
    throw new TypeError('column indices must be an array');
  }

  for (let i = 0; i < columnIndices.length; i++) {
    if (columnIndices[i] < 0 || columnIndices[i] >= matrix.columns) {
      throw new RangeError('column indices are out of range');
    }
  }
}

function checkRange(matrix, startRow, endRow, startColumn, endColumn) {
  if (arguments.length !== 5) {
    throw new RangeError('expected 4 arguments');
  }
  checkNumber('startRow', startRow);
  checkNumber('endRow', endRow);
  checkNumber('startColumn', startColumn);
  checkNumber('endColumn', endColumn);
  if (
    startRow > endRow ||
    startColumn > endColumn ||
    startRow < 0 ||
    startRow >= matrix.rows ||
    endRow < 0 ||
    endRow >= matrix.rows ||
    startColumn < 0 ||
    startColumn >= matrix.columns ||
    endColumn < 0 ||
    endColumn >= matrix.columns
  ) {
    throw new RangeError('Submatrix indices are out of range');
  }
}

function newArray(length, value = 0) {
  let array = [];
  for (let i = 0; i < length; i++) {
    array.push(value);
  }
  return array;
}

function checkNumber(name, value) {
  if (typeof value !== 'number') {
    throw new TypeError(`${name} must be a number`);
  }
}

function checkNonEmpty(matrix) {
  if (matrix.isEmpty()) {
    throw new Error('Empty matrix has no elements to index');
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/base.js":
/*!**************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/base.js ***!
  \**************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ BaseView)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");


class BaseView extends _matrix__WEBPACK_IMPORTED_MODULE_0__.AbstractMatrix {
  constructor(matrix, rows, columns) {
    super();
    this.matrix = matrix;
    this.rows = rows;
    this.columns = columns;
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/column.js":
/*!****************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/column.js ***!
  \****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixColumnView)
/* harmony export */ });
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../util */ "./node_modules/ml-matrix/src/util.js");
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");




class MatrixColumnView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix, column) {
    (0,_util__WEBPACK_IMPORTED_MODULE_1__.checkColumnIndex)(matrix, column);
    super(matrix, matrix.rows, 1);
    this.column = column;
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(rowIndex, this.column, value);
    return this;
  }

  get(rowIndex) {
    return this.matrix.get(rowIndex, this.column);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/columnSelection.js":
/*!*************************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/columnSelection.js ***!
  \*************************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixColumnSelectionView)
/* harmony export */ });
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../util */ "./node_modules/ml-matrix/src/util.js");
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");




class MatrixColumnSelectionView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix, columnIndices) {
    (0,_util__WEBPACK_IMPORTED_MODULE_1__.checkColumnIndices)(matrix, columnIndices);
    super(matrix, matrix.rows, columnIndices.length);
    this.columnIndices = columnIndices;
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(rowIndex, this.columnIndices[columnIndex], value);
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(rowIndex, this.columnIndices[columnIndex]);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/flipColumn.js":
/*!********************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/flipColumn.js ***!
  \********************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixFlipColumnView)
/* harmony export */ });
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");


class MatrixFlipColumnView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix) {
    super(matrix, matrix.rows, matrix.columns);
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(rowIndex, this.columns - columnIndex - 1, value);
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(rowIndex, this.columns - columnIndex - 1);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/flipRow.js":
/*!*****************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/flipRow.js ***!
  \*****************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixFlipRowView)
/* harmony export */ });
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");


class MatrixFlipRowView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix) {
    super(matrix, matrix.rows, matrix.columns);
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(this.rows - rowIndex - 1, columnIndex, value);
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(this.rows - rowIndex - 1, columnIndex);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/index.js":
/*!***************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/index.js ***!
  \***************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "MatrixColumnSelectionView": () => (/* reexport safe */ _columnSelection__WEBPACK_IMPORTED_MODULE_1__["default"]),
/* harmony export */   "MatrixColumnView": () => (/* reexport safe */ _column__WEBPACK_IMPORTED_MODULE_0__["default"]),
/* harmony export */   "MatrixFlipColumnView": () => (/* reexport safe */ _flipColumn__WEBPACK_IMPORTED_MODULE_2__["default"]),
/* harmony export */   "MatrixFlipRowView": () => (/* reexport safe */ _flipRow__WEBPACK_IMPORTED_MODULE_3__["default"]),
/* harmony export */   "MatrixRowSelectionView": () => (/* reexport safe */ _rowSelection__WEBPACK_IMPORTED_MODULE_5__["default"]),
/* harmony export */   "MatrixRowView": () => (/* reexport safe */ _row__WEBPACK_IMPORTED_MODULE_4__["default"]),
/* harmony export */   "MatrixSelectionView": () => (/* reexport safe */ _selection__WEBPACK_IMPORTED_MODULE_6__["default"]),
/* harmony export */   "MatrixSubView": () => (/* reexport safe */ _sub__WEBPACK_IMPORTED_MODULE_7__["default"]),
/* harmony export */   "MatrixTransposeView": () => (/* reexport safe */ _transpose__WEBPACK_IMPORTED_MODULE_8__["default"])
/* harmony export */ });
/* harmony import */ var _column__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./column */ "./node_modules/ml-matrix/src/views/column.js");
/* harmony import */ var _columnSelection__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./columnSelection */ "./node_modules/ml-matrix/src/views/columnSelection.js");
/* harmony import */ var _flipColumn__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./flipColumn */ "./node_modules/ml-matrix/src/views/flipColumn.js");
/* harmony import */ var _flipRow__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./flipRow */ "./node_modules/ml-matrix/src/views/flipRow.js");
/* harmony import */ var _row__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! ./row */ "./node_modules/ml-matrix/src/views/row.js");
/* harmony import */ var _rowSelection__WEBPACK_IMPORTED_MODULE_5__ = __webpack_require__(/*! ./rowSelection */ "./node_modules/ml-matrix/src/views/rowSelection.js");
/* harmony import */ var _selection__WEBPACK_IMPORTED_MODULE_6__ = __webpack_require__(/*! ./selection */ "./node_modules/ml-matrix/src/views/selection.js");
/* harmony import */ var _sub__WEBPACK_IMPORTED_MODULE_7__ = __webpack_require__(/*! ./sub */ "./node_modules/ml-matrix/src/views/sub.js");
/* harmony import */ var _transpose__WEBPACK_IMPORTED_MODULE_8__ = __webpack_require__(/*! ./transpose */ "./node_modules/ml-matrix/src/views/transpose.js");











/***/ }),

/***/ "./node_modules/ml-matrix/src/views/row.js":
/*!*************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/row.js ***!
  \*************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixRowView)
/* harmony export */ });
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../util */ "./node_modules/ml-matrix/src/util.js");
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");




class MatrixRowView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix, row) {
    (0,_util__WEBPACK_IMPORTED_MODULE_1__.checkRowIndex)(matrix, row);
    super(matrix, 1, matrix.columns);
    this.row = row;
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(this.row, columnIndex, value);
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(this.row, columnIndex);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/rowSelection.js":
/*!**********************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/rowSelection.js ***!
  \**********************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixRowSelectionView)
/* harmony export */ });
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../util */ "./node_modules/ml-matrix/src/util.js");
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");




class MatrixRowSelectionView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix, rowIndices) {
    (0,_util__WEBPACK_IMPORTED_MODULE_1__.checkRowIndices)(matrix, rowIndices);
    super(matrix, rowIndices.length, matrix.columns);
    this.rowIndices = rowIndices;
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(this.rowIndices[rowIndex], columnIndex, value);
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(this.rowIndices[rowIndex], columnIndex);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/selection.js":
/*!*******************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/selection.js ***!
  \*******************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixSelectionView)
/* harmony export */ });
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../util */ "./node_modules/ml-matrix/src/util.js");
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");




class MatrixSelectionView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix, rowIndices, columnIndices) {
    (0,_util__WEBPACK_IMPORTED_MODULE_1__.checkRowIndices)(matrix, rowIndices);
    (0,_util__WEBPACK_IMPORTED_MODULE_1__.checkColumnIndices)(matrix, columnIndices);
    super(matrix, rowIndices.length, columnIndices.length);
    this.rowIndices = rowIndices;
    this.columnIndices = columnIndices;
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(
      this.rowIndices[rowIndex],
      this.columnIndices[columnIndex],
      value,
    );
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(
      this.rowIndices[rowIndex],
      this.columnIndices[columnIndex],
    );
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/sub.js":
/*!*************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/sub.js ***!
  \*************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixSubView)
/* harmony export */ });
/* harmony import */ var _util__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ../util */ "./node_modules/ml-matrix/src/util.js");
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");




class MatrixSubView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix, startRow, endRow, startColumn, endColumn) {
    (0,_util__WEBPACK_IMPORTED_MODULE_1__.checkRange)(matrix, startRow, endRow, startColumn, endColumn);
    super(matrix, endRow - startRow + 1, endColumn - startColumn + 1);
    this.startRow = startRow;
    this.startColumn = startColumn;
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(
      this.startRow + rowIndex,
      this.startColumn + columnIndex,
      value,
    );
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(
      this.startRow + rowIndex,
      this.startColumn + columnIndex,
    );
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/views/transpose.js":
/*!*******************************************************!*\
  !*** ./node_modules/ml-matrix/src/views/transpose.js ***!
  \*******************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ MatrixTransposeView)
/* harmony export */ });
/* harmony import */ var _base__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./base */ "./node_modules/ml-matrix/src/views/base.js");


class MatrixTransposeView extends _base__WEBPACK_IMPORTED_MODULE_0__["default"] {
  constructor(matrix) {
    super(matrix, matrix.columns, matrix.rows);
  }

  set(rowIndex, columnIndex, value) {
    this.matrix.set(columnIndex, rowIndex, value);
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.matrix.get(columnIndex, rowIndex);
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/wrap/WrapperMatrix1D.js":
/*!************************************************************!*\
  !*** ./node_modules/ml-matrix/src/wrap/WrapperMatrix1D.js ***!
  \************************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ WrapperMatrix1D)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");


class WrapperMatrix1D extends _matrix__WEBPACK_IMPORTED_MODULE_0__.AbstractMatrix {
  constructor(data, options = {}) {
    const { rows = 1 } = options;

    if (data.length % rows !== 0) {
      throw new Error('the data length is not divisible by the number of rows');
    }
    super();
    this.rows = rows;
    this.columns = data.length / rows;
    this.data = data;
  }

  set(rowIndex, columnIndex, value) {
    let index = this._calculateIndex(rowIndex, columnIndex);
    this.data[index] = value;
    return this;
  }

  get(rowIndex, columnIndex) {
    let index = this._calculateIndex(rowIndex, columnIndex);
    return this.data[index];
  }

  _calculateIndex(row, column) {
    return row * this.columns + column;
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js":
/*!************************************************************!*\
  !*** ./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js ***!
  \************************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ WrapperMatrix2D)
/* harmony export */ });
/* harmony import */ var _matrix__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ../matrix */ "./node_modules/ml-matrix/src/matrix.js");


class WrapperMatrix2D extends _matrix__WEBPACK_IMPORTED_MODULE_0__.AbstractMatrix {
  constructor(data) {
    super();
    this.data = data;
    this.rows = data.length;
    this.columns = data[0].length;
  }

  set(rowIndex, columnIndex, value) {
    this.data[rowIndex][columnIndex] = value;
    return this;
  }

  get(rowIndex, columnIndex) {
    return this.data[rowIndex][columnIndex];
  }
}


/***/ }),

/***/ "./node_modules/ml-matrix/src/wrap/wrap.js":
/*!*************************************************!*\
  !*** ./node_modules/ml-matrix/src/wrap/wrap.js ***!
  \*************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "wrap": () => (/* binding */ wrap)
/* harmony export */ });
/* harmony import */ var is_any_array__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! is-any-array */ "./node_modules/is-any-array/lib-esm/index.js");
/* harmony import */ var _WrapperMatrix1D__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! ./WrapperMatrix1D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix1D.js");
/* harmony import */ var _WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./WrapperMatrix2D */ "./node_modules/ml-matrix/src/wrap/WrapperMatrix2D.js");





function wrap(array, options) {
  if ((0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(array)) {
    if (array[0] && (0,is_any_array__WEBPACK_IMPORTED_MODULE_0__.isAnyArray)(array[0])) {
      return new _WrapperMatrix2D__WEBPACK_IMPORTED_MODULE_1__["default"](array);
    } else {
      return new _WrapperMatrix1D__WEBPACK_IMPORTED_MODULE_2__["default"](array, options);
    }
  } else {
    throw new Error('the argument is not an array');
  }
}


/***/ }),

/***/ "./node_modules/tinyqueue/index.js":
/*!*****************************************!*\
  !*** ./node_modules/tinyqueue/index.js ***!
  \*****************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
__webpack_require__.r(__webpack_exports__);
/* harmony export */ __webpack_require__.d(__webpack_exports__, {
/* harmony export */   "default": () => (/* binding */ TinyQueue)
/* harmony export */ });

class TinyQueue {
    constructor(data = [], compare = defaultCompare) {
        this.data = data;
        this.length = this.data.length;
        this.compare = compare;

        if (this.length > 0) {
            for (let i = (this.length >> 1) - 1; i >= 0; i--) this._down(i);
        }
    }

    push(item) {
        this.data.push(item);
        this.length++;
        this._up(this.length - 1);
    }

    pop() {
        if (this.length === 0) return undefined;

        const top = this.data[0];
        const bottom = this.data.pop();
        this.length--;

        if (this.length > 0) {
            this.data[0] = bottom;
            this._down(0);
        }

        return top;
    }

    peek() {
        return this.data[0];
    }

    _up(pos) {
        const {data, compare} = this;
        const item = data[pos];

        while (pos > 0) {
            const parent = (pos - 1) >> 1;
            const current = data[parent];
            if (compare(item, current) >= 0) break;
            data[pos] = current;
            pos = parent;
        }

        data[pos] = item;
    }

    _down(pos) {
        const {data, compare} = this;
        const halfLength = this.length >> 1;
        const item = data[pos];

        while (pos < halfLength) {
            let left = (pos << 1) + 1;
            let best = data[left];
            const right = left + 1;

            if (right < this.length && compare(data[right], best) < 0) {
                left = right;
                best = data[right];
            }
            if (compare(best, item) >= 0) break;

            data[pos] = best;
            pos = left;
        }

        data[pos] = item;
    }
}

function defaultCompare(a, b) {
    return a < b ? -1 : a > b ? 1 : 0;
}


/***/ })

/******/ 	});
/************************************************************************/
/******/ 	// The module cache
/******/ 	var __webpack_module_cache__ = {};
/******/ 	
/******/ 	// The require function
/******/ 	function __webpack_require__(moduleId) {
/******/ 		// Check if module is in cache
/******/ 		var cachedModule = __webpack_module_cache__[moduleId];
/******/ 		if (cachedModule !== undefined) {
/******/ 			return cachedModule.exports;
/******/ 		}
/******/ 		// Create a new module (and put it into the cache)
/******/ 		var module = __webpack_module_cache__[moduleId] = {
/******/ 			// no module.id needed
/******/ 			// no module.loaded needed
/******/ 			exports: {}
/******/ 		};
/******/ 	
/******/ 		// Execute the module function
/******/ 		__webpack_modules__[moduleId](module, module.exports, __webpack_require__);
/******/ 	
/******/ 		// Return the exports of the module
/******/ 		return module.exports;
/******/ 	}
/******/ 	
/************************************************************************/
/******/ 	/* webpack/runtime/define property getters */
/******/ 	(() => {
/******/ 		// define getter functions for harmony exports
/******/ 		__webpack_require__.d = (exports, definition) => {
/******/ 			for(var key in definition) {
/******/ 				if(__webpack_require__.o(definition, key) && !__webpack_require__.o(exports, key)) {
/******/ 					Object.defineProperty(exports, key, { enumerable: true, get: definition[key] });
/******/ 				}
/******/ 			}
/******/ 		};
/******/ 	})();
/******/ 	
/******/ 	/* webpack/runtime/hasOwnProperty shorthand */
/******/ 	(() => {
/******/ 		__webpack_require__.o = (obj, prop) => (Object.prototype.hasOwnProperty.call(obj, prop))
/******/ 	})();
/******/ 	
/******/ 	/* webpack/runtime/make namespace object */
/******/ 	(() => {
/******/ 		// define __esModule on exports
/******/ 		__webpack_require__.r = (exports) => {
/******/ 			if(typeof Symbol !== 'undefined' && Symbol.toStringTag) {
/******/ 				Object.defineProperty(exports, Symbol.toStringTag, { value: 'Module' });
/******/ 			}
/******/ 			Object.defineProperty(exports, '__esModule', { value: true });
/******/ 		};
/******/ 	})();
/******/ 	
/************************************************************************/
var __webpack_exports__ = {};
// This entry need to be wrapped in an IIFE because it need to be isolated against other modules in the chunk.
(() => {
/*!**************************************************!*\
  !*** ./mindar/image-target/controller.worker.js ***!
  \**************************************************/
const {Matcher} = __webpack_require__(/*! ./matching/matcher.js */ "./mindar/image-target/matching/matcher.js");
const {Estimator} = __webpack_require__(/*! ./estimation/estimator.js */ "./mindar/image-target/estimation/estimator.js");

let projectionTransform = null;
let matchingDataList = null;
let debugMode = false;
let matcher = null;
let estimator = null;

onmessage = (msg) => {
  const {data} = msg;

  if (data.type === 'setup') {
    projectionTransform = data.projectionTransform;
    matchingDataList = data.matchingDataList;
    debugMode = data.debugMode;
    matcher = new Matcher(data.inputWidth, data.inputHeight, debugMode);
    estimator = new Estimator(data.projectionTransform);
  }
  else if (data.type === 'match') {
    const interestedTargetIndexes = data.targetIndexes;

    let matchedTargetIndex = -1;
    let matchedModelViewTransform = null;
    let matchedDebugExtra = null;

    for (let i = 0; i < interestedTargetIndexes.length; i++) {
      const matchingIndex = interestedTargetIndexes[i];

      const {keyframeIndex, screenCoords, worldCoords, debugExtra} = matcher.matchDetection(matchingDataList[matchingIndex], data.featurePoints);
      matchedDebugExtra = debugExtra;

      if (keyframeIndex !== -1) {
	const modelViewTransform = estimator.estimate({screenCoords, worldCoords});

	if (modelViewTransform) {
	  matchedTargetIndex = matchingIndex;
	  matchedModelViewTransform = modelViewTransform;
	}
	break;
      }
    }

    postMessage({
      type: 'matchDone',
      targetIndex: matchedTargetIndex,
      modelViewTransform: matchedModelViewTransform,
      debugExtra: matchedDebugExtra
    });
  }
  else if (data.type === 'trackUpdate') {
    const {modelViewTransform, worldCoords, screenCoords} = data;
    const finalModelViewTransform = estimator.refineEstimate({initialModelViewTransform: modelViewTransform, worldCoords, screenCoords});
    postMessage({
      type: 'trackUpdateDone',
      modelViewTransform: finalModelViewTransform,
    });
  }
};


})();

/******/ })()
;
//# sourceMappingURL=data:application/json;charset=utf-8;base64,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QUFDQTs7QUFFQTtBQUNBO0FBQ0E7Ozs7Ozs7Ozs7O0FDbktBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOzs7Ozs7Ozs7Ozs7QUNwSEEsT0FBTyxpQkFBaUIsRUFBRSxtQkFBTyxDQUFDLHdEQUFXOztBQUU3QztBQUNBLFNBQVMsNENBQTRDO0FBQ3JELFNBQVMsNENBQTRDOztBQUVyRDtBQUNBO0FBQ0E7QUFDQSxrQkFBa0IsU0FBUztBQUMzQjtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSxJQUFJO0FBQ0o7QUFDQTtBQUNBOztBQUVB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QUFDQTtBQUNBOztBQUVBO0FBQ0Esb0JBQW9CLGVBQWU7QUFDbkMsc0JBQXNCLGtCQUFrQjtBQUN4QztBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBLG9CQUFvQixlQUFlO0FBQ25DLHNCQUFzQixrQkFBa0I7QUFDeEM7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0Esb0JBQW9CLGVBQWU7QUFDbkMsc0JBQXNCLGtCQUFrQjtBQUN4QztBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBLG9CQUFvQixlQUFlO0FBQ25DLHNCQUFzQixrQkFBa0I7QUFDeEM7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0Esb0JBQW9CLGVBQWU7QUFDbkMsc0JBQXNCLGtCQUFrQjtBQUN4QztBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBLG9CQUFvQixlQUFlO0FBQ25DLHNCQUFzQixrQkFBa0I7QUFDeEM7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBLG9CQUFvQixlQUFlO0FBQ25DLHNCQUFzQixrQkFBa0I7QUFDeEM7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0Esb0JBQW9CLGVBQWU7QUFDbkMsc0JBQXNCLGtCQUFrQjtBQUN4QztBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0Esb0JBQW9CLGVBQWU7QUFDbkMsc0JBQXNCLGtCQUFrQjtBQUN4QztBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBLG9CQUFvQixlQUFlO0FBQ25DLHNCQUFzQixrQkFBa0I7QUFDeEM7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBLG9CQUFvQixlQUFlO0FBQ25DLHNCQUFzQixrQkFBa0I7QUFDeEM7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0Esb0JBQW9CLGVBQWU7QUFDbkMsc0JBQXNCLGtCQUFrQjtBQUN4QztBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQUVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQSxvQkFBb0IsZUFBZTtBQUNuQyxzQkFBc0Isa0JBQWtCO0FBQ3hDO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0Esb0JBQW9CLGVBQWU7QUFDbkMsc0JBQXNCLGtCQUFrQjtBQUN4QztBQUNBO0FBQ0E7QUFDQTtB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O0FBQ0EsS0FBSztBQUNMO0FBQ0E7QUFDQSxXQUFXLCtDQUErQztBQUMxRCw4REFBOEQseUVBQXlFO0FBQ3ZJO0FBQ0E7QUFDQTtBQUNBLEtBQUs7QUFDTDtBQUNBIiwic291cmNlcyI6WyJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL21pbmRhci9pbWFnZS10YXJnZXQvZXN0aW1hdGlvbi9lc3RpbWF0ZS5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbWluZGFyL2ltYWdlLXRhcmdldC9lc3RpbWF0aW9uL2VzdGltYXRvci5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbWluZGFyL2ltYWdlLXRhcmdldC9lc3RpbWF0aW9uL3JlZmluZS1lc3RpbWF0ZS5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbWluZGFyL2ltYWdlLXRhcmdldC9lc3RpbWF0aW9uL3V0aWxzLmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9taW5kYXIvaW1hZ2UtdGFyZ2V0L21hdGNoaW5nL2hhbW1pbmctZGlzdGFuY2UuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL21pbmRhci9pbWFnZS10YXJnZXQvbWF0Y2hpbmcvaG91Z2guanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL21pbmRhci9pbWFnZS10YXJnZXQvbWF0Y2hpbmcvbWF0Y2hlci5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbWluZGFyL2ltYWdlLXRhcmdldC9tYXRjaGluZy9tYXRjaGluZy5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbWluZGFyL2ltYWdlLXRhcmdldC9tYXRjaGluZy9yYW5zYWNIb21vZ3JhcGh5LmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9taW5kYXIvaW1hZ2UtdGFyZ2V0L3V0aWxzL2dlb21ldHJ5LmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9taW5kYXIvaW1hZ2UtdGFyZ2V0L3V0aWxzL2hvbW9ncmFwaHkuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL21pbmRhci9pbWFnZS10YXJnZXQvdXRpbHMvcmFuZG9taXplci5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL2lzLWFueS1hcnJheS9saWItZXNtL2luZGV4LmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtYXJyYXktbWF4L2xpYi1lczYvaW5kZXguanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1hcnJheS1taW4vbGliLWVzNi9pbmRleC5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLWFycmF5LXJlc2NhbGUvbGliLWVzNi9pbmRleC5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvY29ycmVsYXRpb24uanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2NvdmFyaWFuY2UuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2RjL2Nob2xlc2t5LmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy9kYy9ldmQuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2RjL2x1LmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy9kYy9uaXBhbHMuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2RjL3FyLmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy9kYy9zdmQuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2RjL3V0aWwuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2RlY29tcG9zaXRpb25zLmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy9kZXRlcm1pbmFudC5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvaW5kZXguanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2luc3BlY3QuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL2xpbmVhckRlcGVuZGVuY2llcy5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvbWF0aE9wZXJhdGlvbnMuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL21hdHJpeC5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvcHNldWRvSW52ZXJzZS5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvc3RhdC5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvdXRpbC5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvdmlld3MvYmFzZS5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvdmlld3MvY29sdW1uLmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy92aWV3cy9jb2x1bW5TZWxlY3Rpb24uanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL3ZpZXdzL2ZsaXBDb2x1bW4uanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL3ZpZXdzL2ZsaXBSb3cuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL3ZpZXdzL2luZGV4LmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy92aWV3cy9yb3cuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL3ZpZXdzL3Jvd1NlbGVjdGlvbi5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvdmlld3Mvc2VsZWN0aW9uLmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy92aWV3cy9zdWIuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL3ZpZXdzL3RyYW5zcG9zZS5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbm9kZV9tb2R1bGVzL21sLW1hdHJpeC9zcmMvd3JhcC9XcmFwcGVyTWF0cml4MUQuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy9tbC1tYXRyaXgvc3JjL3dyYXAvV3JhcHBlck1hdHJpeDJELmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9ub2RlX21vZHVsZXMvbWwtbWF0cml4L3NyYy93cmFwL3dyYXAuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL25vZGVfbW9kdWxlcy90aW55cXVldWUvaW5kZXguanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC93ZWJwYWNrL2Jvb3RzdHJhcCIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwL3dlYnBhY2svcnVudGltZS9kZWZpbmUgcHJvcGVydHkgZ2V0dGVycyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwL3dlYnBhY2svcnVudGltZS9oYXNPd25Qcm9wZXJ0eSBzaG9ydGhhbmQiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC93ZWJwYWNrL3J1bnRpbWUvbWFrZSBuYW1lc3BhY2Ugb2JqZWN0Iiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9taW5kYXIvaW1hZ2UtdGFyZ2V0L2NvbnRyb2xsZXIud29ya2VyLmpzIl0sInNvdXJjZXNDb250ZW50IjpbImNvbnN0IHtNYXRyaXgsIGludmVyc2V9ID0gcmVxdWlyZSgnbWwtbWF0cml4Jyk7XG5jb25zdCB7c29sdmVIb21vZ3JhcGh5fSA9IHJlcXVpcmUoJy4uL3V0aWxzL2hvbW9ncmFwaHknKTtcblxuLy8gYnVpbGQgd29ybGQgbWF0cml4IHdpdGggbGlzdCBvZiBtYXRjaGluZyB3b3JsZENvb3Jkc3xzY3JlZW5Db29yZHNcbi8vXG4vLyBTdGVwIDEuIGVzdGltYXRlIGhvbW9ncmFwaHkgd2l0aCBsaXN0IG9mIHBhaXJzXG4vLyBSZWY6IGh0dHBzOi8vd3d3LnVpby5uby9zdHVkaWVyL2VtbmVyL21hdG5hdC9pdHMvVEVLNTAzMC92MTkvbGVjdC9sZWN0dXJlXzRfMy1lc3RpbWF0aW5nLWhvbW9ncmFwaGllcy1mcm9tLWZlYXR1cmUtY29ycmVzcG9uZGVuY2VzLnBkZiAgKEJhc2ljIGhvbW9ncmFwaHkgZXN0aW1hdGlvbiBmcm9tIHBvaW50cylcbi8vXG4vLyBTdGVwIDIuIGRlY29tcG9zZSBob21vZ3JhcGh5IGludG8gcm90YXRpb24gYW5kIHRyYW5zbGF0aW9uIG1hdHJpeGVzIChpLmUuIHdvcmxkIG1hdHJpeClcbi8vIFJlZjogY2FuIGFueW9uZSBwcm92aWRlIHJlZmVyZW5jZT9cbmNvbnN0IGVzdGltYXRlID0gKHtzY3JlZW5Db29yZHMsIHdvcmxkQ29vcmRzLCBwcm9qZWN0aW9uVHJhbnNmb3JtfSkgPT4ge1xuICBjb25zdCBIYXJyYXkgPSBzb2x2ZUhvbW9ncmFwaHkod29ybGRDb29yZHMubWFwKChwKSA9PiBbcC54LCBwLnldKSwgc2NyZWVuQ29vcmRzLm1hcCgocCkgPT4gW3AueCwgcC55XSkpO1xuICBjb25zdCBIID0gbmV3IE1hdHJpeChbXG4gICAgW0hhcnJheVswXSwgSGFycmF5WzFdLCBIYXJyYXlbMl1dLFxuICAgIFtIYXJyYXlbM10sIEhhcnJheVs0XSwgSGFycmF5WzVdXSxcbiAgICBbSGFycmF5WzZdLCBIYXJyYXlbN10sIEhhcnJheVs4XV0sXG4gIF0pO1xuXG4gIGNvbnN0IEsgPSBuZXcgTWF0cml4KHByb2plY3Rpb25UcmFuc2Zvcm0pO1xuICBjb25zdCBLSW52ID0gaW52ZXJzZShLKTtcblxuICBjb25zdCBfS0ludkggPSBLSW52Lm1tdWwoSCk7XG4gIGNvbnN0IEtJbnZIID0gX0tJbnZILnRvMURBcnJheSgpO1xuXG4gIGNvbnN0IG5vcm0xID0gTWF0aC5zcXJ0KCBLSW52SFswXSAqIEtJbnZIWzBdICsgS0ludkhbM10gKiBLSW52SFszXSArIEtJbnZIWzZdICogS0ludkhbNl0pO1xuICBjb25zdCBub3JtMiA9IE1hdGguc3FydCggS0ludkhbMV0gKiBLSW52SFsxXSArIEtJbnZIWzRdICogS0ludkhbNF0gKyBLSW52SFs3XSAqIEtJbnZIWzddKTtcbiAgY29uc3QgdG5vcm0gPSAobm9ybTEgKyBub3JtMikgLyAyO1xuXG4gIGNvbnN0IHJvdGF0ZSA9IFtdO1xuICByb3RhdGVbMF0gPSBLSW52SFswXSAvIG5vcm0xO1xuICByb3RhdGVbM10gPSBLSW52SFszXSAvIG5vcm0xO1xuICByb3RhdGVbNl0gPSBLSW52SFs2XSAvIG5vcm0xO1xuXG4gIHJvdGF0ZVsxXSA9IEtJbnZIWzFdIC8gbm9ybTI7XG4gIHJvdGF0ZVs0XSA9IEtJbnZIWzRdIC8gbm9ybTI7XG4gIHJvdGF0ZVs3XSA9IEtJbnZIWzddIC8gbm9ybTI7XG5cbiAgcm90YXRlWzJdID0gcm90YXRlWzNdICogcm90YXRlWzddIC0gcm90YXRlWzZdICogcm90YXRlWzRdO1xuICByb3RhdGVbNV0gPSByb3RhdGVbNl0gKiByb3RhdGVbMV0gLSByb3RhdGVbMF0gKiByb3RhdGVbN107XG4gIHJvdGF0ZVs4XSA9IHJvdGF0ZVswXSAqIHJvdGF0ZVs0XSAtIHJvdGF0ZVsxXSAqIHJvdGF0ZVszXTtcblxuICBjb25zdCBub3JtMyA9IE1hdGguc3FydChyb3RhdGVbMl0gKiByb3RhdGVbMl0gKyByb3RhdGVbNV0gKiByb3RhdGVbNV0gKyByb3RhdGVbOF0gKiByb3RhdGVbOF0pO1xuICByb3RhdGVbMl0gLz0gbm9ybTM7XG4gIHJvdGF0ZVs1XSAvPSBub3JtMztcbiAgcm90YXRlWzhdIC89IG5vcm0zO1xuXG4gIC8vIFRPRE86IGFydG9vbGtpdCBoYXMgY2hlY2tfcm90YXRpb24oKSB0aGF0IHNvbWVob3cgc3dpdGNoIHRoZSByb3RhdGUgdmVjdG9yLiBub3Qgc3VyZSB3aGF0IHRoYXQgZG9lcy4gQ2FuIGFueW9uZSBhZHZpY2U/XG4gIC8vIGh0dHBzOi8vZ2l0aHViLmNvbS9hcnRvb2xraXR4L2FydG9vbGtpdDUvYmxvYi81YmYwYjY3MWZmMTZlYWQ1MjdiOWI4OTJlNmFlYjFhMjc3MWY5N2JlL2xpYi9TUkMvQVJJQ1AvaWNwVXRpbC5jI0wyMTVcblxuICBjb25zdCB0cmFuID0gW11cbiAgdHJhblswXSA9IEtJbnZIWzJdIC8gdG5vcm07XG4gIHRyYW5bMV0gPSBLSW52SFs1XSAvIHRub3JtO1xuICB0cmFuWzJdID0gS0ludkhbOF0gLyB0bm9ybTtcblxuICBsZXQgaW5pdGlhbE1vZGVsVmlld1RyYW5zZm9ybSA9IFtcbiAgICBbcm90YXRlWzBdLCByb3RhdGVbMV0sIHJvdGF0ZVsyXSwgdHJhblswXV0sXG4gICAgW3JvdGF0ZVszXSwgcm90YXRlWzRdLCByb3RhdGVbNV0sIHRyYW5bMV1dLFxuICAgIFtyb3RhdGVbNl0sIHJvdGF0ZVs3XSwgcm90YXRlWzhdLCB0cmFuWzJdXVxuICBdO1xuXG4gIHJldHVybiBpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSB7XG4gIGVzdGltYXRlXG59XG4iLCJjb25zdCB7ZXN0aW1hdGV9ID0gcmVxdWlyZSgnLi9lc3RpbWF0ZS5qcycpO1xuY29uc3Qge3JlZmluZUVzdGltYXRlfSA9IHJlcXVpcmUoJy4vcmVmaW5lLWVzdGltYXRlLmpzJyk7XG5cbmNsYXNzIEVzdGltYXRvciB7XG4gIGNvbnN0cnVjdG9yKHByb2plY3Rpb25UcmFuc2Zvcm0pIHtcbiAgICB0aGlzLnByb2plY3Rpb25UcmFuc2Zvcm0gPSBwcm9qZWN0aW9uVHJhbnNmb3JtO1xuICB9XG5cbiAgLy8gU29sdmUgaG9tb2dyYXBoeSBiZXR3ZWVuIHNjcmVlbiBwb2ludHMgYW5kIHdvcmxkIHBvaW50cyB1c2luZyBEaXJlY3QgTGluZWFyIFRyYW5zZm9ybWF0aW9uXG4gIC8vIHRoZW4gZGVjb21wb3NlIGhvbW9ncmFwaHkgaW50byByb3RhdGlvbiBhbmQgdHJhbnNsYXRpb24gbWF0cml4IChpLmUuIG1vZGVsVmlld1RyYW5zZm9ybSlcbiAgZXN0aW1hdGUoe3NjcmVlbkNvb3Jkcywgd29ybGRDb29yZHN9KSB7XG4gICAgY29uc3QgbW9kZWxWaWV3VHJhbnNmb3JtID0gZXN0aW1hdGUoe3NjcmVlbkNvb3Jkcywgd29ybGRDb29yZHMsIHByb2plY3Rpb25UcmFuc2Zvcm06IHRoaXMucHJvamVjdGlvblRyYW5zZm9ybX0pO1xuICAgIHJldHVybiBtb2RlbFZpZXdUcmFuc2Zvcm07XG4gIH1cblxuICAvLyBHaXZlbiBhbiBpbml0aWFsIGd1ZXNzIG9mIHRoZSBtb2RlbFZpZXdUcmFuc2Zvcm0gYW5kIG5ldyBwYWlycyBvZiBzY3JlZW4td29ybGQgY29vcmRpbmF0ZXMsIFxuICAvLyB1c2UgSXRlcmF0aXZlIENsb3Nlc3QgUG9pbnQgdG8gcmVmaW5lIHRoZSB0cmFuc2Zvcm1hdGlvblxuICAvL3JlZmluZUVzdGltYXRlKHtpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtLCBzY3JlZW5Db29yZHMsIHdvcmxkQ29vcmRzfSkge1xuICByZWZpbmVFc3RpbWF0ZSh7aW5pdGlhbE1vZGVsVmlld1RyYW5zZm9ybSwgd29ybGRDb29yZHMsIHNjcmVlbkNvb3Jkc30pIHtcbiAgICBjb25zdCB1cGRhdGVkTW9kZWxWaWV3VHJhbnNmb3JtID0gcmVmaW5lRXN0aW1hdGUoe2luaXRpYWxNb2RlbFZpZXdUcmFuc2Zvcm0sIHdvcmxkQ29vcmRzLCBzY3JlZW5Db29yZHMsIHByb2plY3Rpb25UcmFuc2Zvcm06IHRoaXMucHJvamVjdGlvblRyYW5zZm9ybX0pO1xuICAgIHJldHVybiB1cGRhdGVkTW9kZWxWaWV3VHJhbnNmb3JtO1xuICB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuICBFc3RpbWF0b3IsXG59XG4iLCJjb25zdCB7TWF0cml4LCBpbnZlcnNlfSA9IHJlcXVpcmUoJ21sLW1hdHJpeCcpO1xuY29uc3Qge25vcm1hbGl6ZVBvaW50cywgYXBwbHlNb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtLCBidWlsZE1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm0sIGNvbXB1dGVTY3JlZW5Db29yZGlhdGV9ID0gcmVxdWlyZSgnLi91dGlscy5qcycpO1xuXG5jb25zdCBUUkFDS0lOR19USFJFU0ggPSA1LjA7IC8vIGRlZmF1bHRcbmNvbnN0IEsyX0ZBQ1RPUiA9IDQuMDsgLy8gUXVlc3Rpb246IHNob3VsZCBpdCBiZSByZWxhdGl2ZSB0byB0aGUgc2l6ZSBvZiB0aGUgc2NyZWVuIGluc3RlYWQgb2YgaGFyZGNvZGVkP1xuY29uc3QgSUNQX01BWF9MT09QID0gMTA7XG5jb25zdCBJQ1BfQlJFQUtfTE9PUF9FUlJPUl9USFJFU0ggPSAwLjE7XG5jb25zdCBJQ1BfQlJFQUtfTE9PUF9FUlJPUl9SQVRJT19USFJFU0ggPSAwLjk5O1xuY29uc3QgSUNQX0JSRUFLX0xPT1BfRVJST1JfVEhSRVNIMiA9IDQuMDtcblxuLy8gc29tZSB0ZW1wb3JhcnkvaW50ZXJtZWRpYXRlIHZhcmlhYmxlcyB1c2VkIGxhdGVyLiBEZWNsYXJlIHRoZW0gYmVmb3JlaGFuZCB0byByZWR1Y2UgbmV3IG9iamVjdCBhbGxvY2F0aW9uc1xubGV0IG1hdCA9IFtbXSxbXSxbXV07IFxubGV0IEpfVV9YYyA9IFtbXSxbXV07IC8vIDJ4M1xubGV0IEpfWGNfUyA9IFtbXSxbXSxbXV07IC8vIDN4NlxuXG5jb25zdCByZWZpbmVFc3RpbWF0ZSA9ICh7aW5pdGlhbE1vZGVsVmlld1RyYW5zZm9ybSwgcHJvamVjdGlvblRyYW5zZm9ybSwgd29ybGRDb29yZHMsIHNjcmVlbkNvb3Jkc30pID0+IHtcbiAgLy8gUXVlc3Rpb246IHNoYWxsIHdlIG5vcm1saXplIHRoZSBzY3JlZW4gY29vcmRzIGFzIHdlbGw/XG4gIC8vIFF1ZXN0aW9uOiBkbyB3ZSBuZWVkIHRvIG5vcm1saXplIHRoZSBzY2FsZSBhcyB3ZWxsLCBpLmUuIG1ha2UgY29vcmRzIGZyb20gLTEgdG8gMVxuICAvL1xuICAvLyBub3JtYWxpemUgd29ybGQgY29vcmRzIC0gcmVwb3NpdGlvbiB0aGVtIHRvIGNlbnRlciBvZiBtYXNzXG4gIC8vICAgYXNzdW1lIHogY29vcmRpbmF0ZSBpcyBhbHdheXMgemVybyAoaW4gb3VyIGNhc2UsIHRoZSBpbWFnZSB0YXJnZXQgaXMgcGxhbmFyIHdpdGggeiA9IDBcbiAgbGV0IGR4ID0gMDtcbiAgbGV0IGR5ID0gMDtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCB3b3JsZENvb3Jkcy5sZW5ndGg7IGkrKykge1xuICAgIGR4ICs9IHdvcmxkQ29vcmRzW2ldLng7XG4gICAgZHkgKz0gd29ybGRDb29yZHNbaV0ueTtcbiAgfVxuICBkeCAvPSB3b3JsZENvb3Jkcy5sZW5ndGg7XG4gIGR5IC89IHdvcmxkQ29vcmRzLmxlbmd0aDtcblxuICBjb25zdCBub3JtYWxpemVkV29ybGRDb29yZHMgPSBbXTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCB3b3JsZENvb3Jkcy5sZW5ndGg7IGkrKykge1xuICAgIG5vcm1hbGl6ZWRXb3JsZENvb3Jkcy5wdXNoKHt4OiB3b3JsZENvb3Jkc1tpXS54IC0gZHgsIHk6IHdvcmxkQ29vcmRzW2ldLnkgLSBkeSwgejogd29ybGRDb29yZHNbaV0uen0pO1xuICB9XG5cbiAgY29uc3QgZGlmZk1vZGVsVmlld1RyYW5zZm9ybSA9IFtbXSxbXSxbXV07XG4gIGZvciAobGV0IGogPSAwOyBqIDwgMzsgaisrKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCAzOyBpKyspIHtcbiAgICAgIGRpZmZNb2RlbFZpZXdUcmFuc2Zvcm1bal1baV0gPSBpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtW2pdW2ldO1xuICAgIH1cbiAgfVxuICBkaWZmTW9kZWxWaWV3VHJhbnNmb3JtWzBdWzNdID0gaW5pdGlhbE1vZGVsVmlld1RyYW5zZm9ybVswXVswXSAqIGR4ICsgaW5pdGlhbE1vZGVsVmlld1RyYW5zZm9ybVswXVsxXSAqIGR5ICsgaW5pdGlhbE1vZGVsVmlld1RyYW5zZm9ybVswXVszXTtcbiAgZGlmZk1vZGVsVmlld1RyYW5zZm9ybVsxXVszXSA9IGluaXRpYWxNb2RlbFZpZXdUcmFuc2Zvcm1bMV1bMF0gKiBkeCArIGluaXRpYWxNb2RlbFZpZXdUcmFuc2Zvcm1bMV1bMV0gKiBkeSArIGluaXRpYWxNb2RlbFZpZXdUcmFuc2Zvcm1bMV1bM107XG4gIGRpZmZNb2RlbFZpZXdUcmFuc2Zvcm1bMl1bM10gPSBpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtWzJdWzBdICogZHggKyBpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtWzJdWzFdICogZHkgKyBpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtWzJdWzNdO1xuXG4gIC8vIHVzZSBpdGVyYXRpdmUgY2xvc2VzdCBwb2ludCBhbGdvcml0aG0gdG8gcmVmaW5lIHRoZSBtb2RlbFZpZXdUcmFuc2Zvcm1cbiAgY29uc3QgaW5saWVyUHJvYnMgPSBbMS4wLCAwLjgsIDAuNiwgMC40LCAwLjBdO1xuICBsZXQgdXBkYXRlZE1vZGVsVmlld1RyYW5zZm9ybSA9IGRpZmZNb2RlbFZpZXdUcmFuc2Zvcm07IC8vIGl0ZXJhdGl2ZWx5IHVwZGF0ZSB0aGlzIHRyYW5zZm9ybVxuICBsZXQgZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm0gPSBudWxsO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGlubGllclByb2JzLmxlbmd0aDsgaSsrKSB7XG4gICAgY29uc3QgcmV0ID0gX2RvSUNQKHtpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtOiB1cGRhdGVkTW9kZWxWaWV3VHJhbnNmb3JtLCBwcm9qZWN0aW9uVHJhbnNmb3JtLCB3b3JsZENvb3Jkczogbm9ybWFsaXplZFdvcmxkQ29vcmRzLCBzY3JlZW5Db29yZHMsIGlubGllclByb2I6IGlubGllclByb2JzW2ldfSk7XG5cbiAgICB1cGRhdGVkTW9kZWxWaWV3VHJhbnNmb3JtID0gcmV0Lm1vZGVsVmlld1RyYW5zZm9ybTtcblxuICAgIC8vY29uc29sZS5sb2coXCJlcnJcIiwgcmV0LmVycik7XG5cbiAgICBpZiAocmV0LmVyciA8IFRSQUNLSU5HX1RIUkVTSCkge1xuICAgICAgZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm0gPSB1cGRhdGVkTW9kZWxWaWV3VHJhbnNmb3JtO1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9XG5cbiAgaWYgKGZpbmFsTW9kZWxWaWV3VHJhbnNmb3JtID09PSBudWxsKSByZXR1cm4gbnVsbDtcblxuICAvLyBkZS1ub3JtYWxpemVcbiAgZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm1bMF1bM10gPSBmaW5hbE1vZGVsVmlld1RyYW5zZm9ybVswXVszXSAtIGZpbmFsTW9kZWxWaWV3VHJhbnNmb3JtWzBdWzBdICogZHggLSBmaW5hbE1vZGVsVmlld1RyYW5zZm9ybVswXVsxXSAqIGR5O1xuICBmaW5hbE1vZGVsVmlld1RyYW5zZm9ybVsxXVszXSA9IGZpbmFsTW9kZWxWaWV3VHJhbnNmb3JtWzFdWzNdIC0gZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm1bMV1bMF0gKiBkeCAtIGZpbmFsTW9kZWxWaWV3VHJhbnNmb3JtWzFdWzFdICogZHk7XG4gIGZpbmFsTW9kZWxWaWV3VHJhbnNmb3JtWzJdWzNdID0gZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm1bMl1bM10gLSBmaW5hbE1vZGVsVmlld1RyYW5zZm9ybVsyXVswXSAqIGR4IC0gZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm1bMl1bMV0gKiBkeTtcblxuICByZXR1cm4gZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm07XG59XG5cbi8vIElDUCBpdGVyYXRpb25cbi8vIFF1ZXN0aW9uOiBjYW4gc29tZW9uZSBwcm92aWRlIHRoZW9yZXRpY2FsIHJlZmVyZW5jZSAvIG1hdGhlbWF0aWNhbCBwcm9vZiBmb3IgdGhlIGZvbGxvd2luZyBjb21wdXRhdGlvbnM/XG5jb25zdCBfZG9JQ1AgPSAoe2luaXRpYWxNb2RlbFZpZXdUcmFuc2Zvcm0sIHByb2plY3Rpb25UcmFuc2Zvcm0sIHdvcmxkQ29vcmRzLCBzY3JlZW5Db29yZHMsIGlubGllclByb2J9KSA9PiB7XG4gIGNvbnN0IGlzUm9idXN0TW9kZSA9IGlubGllclByb2IgPCAxO1xuXG4gIGxldCBtb2RlbFZpZXdUcmFuc2Zvcm0gPSBpbml0aWFsTW9kZWxWaWV3VHJhbnNmb3JtO1xuXG4gIGxldCBlcnIwID0gMC4wO1xuICBsZXQgZXJyMSA9IDAuMDtcblxuICBsZXQgRSA9IG5ldyBBcnJheSh3b3JsZENvb3Jkcy5sZW5ndGgpO1xuICBsZXQgRTIgPSBuZXcgQXJyYXkod29ybGRDb29yZHMubGVuZ3RoKTtcbiAgbGV0IGR4cyA9IG5ldyBBcnJheSh3b3JsZENvb3Jkcy5sZW5ndGgpO1xuICBsZXQgZHlzID0gbmV3IEFycmF5KHdvcmxkQ29vcmRzLmxlbmd0aCk7XG5cbiAgZm9yIChsZXQgbCA9IDA7IGwgPD0gSUNQX01BWF9MT09QOyBsKyspIHtcbiAgICBjb25zdCBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtID0gYnVpbGRNb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtKHByb2plY3Rpb25UcmFuc2Zvcm0sIG1vZGVsVmlld1RyYW5zZm9ybSk7XG5cbiAgICBmb3IgKGxldCBuID0gMDsgbiA8IHdvcmxkQ29vcmRzLmxlbmd0aDsgbisrKSB7XG4gICAgICBjb25zdCB1ID0gY29tcHV0ZVNjcmVlbkNvb3JkaWF0ZShtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtLCB3b3JsZENvb3Jkc1tuXS54LCB3b3JsZENvb3Jkc1tuXS55LCB3b3JsZENvb3Jkc1tuXS56KTtcbiAgICAgIGNvbnN0IGR4ID0gc2NyZWVuQ29vcmRzW25dLnggLSB1Lng7XG4gICAgICBjb25zdCBkeSA9IHNjcmVlbkNvb3Jkc1tuXS55IC0gdS55O1xuXG4gICAgICBkeHNbbl0gPSBkeDtcbiAgICAgIGR5c1tuXSA9IGR5O1xuICAgICAgRVtuXSA9IChkeCAqIGR4ICsgZHkgKiBkeSk7XG4gICAgfVxuXG4gICAgbGV0IEsyOyAvLyByb2J1c3QgbW9kZSBvbmx5XG4gICAgZXJyMSA9IDAuMDtcbiAgICBpZiAoaXNSb2J1c3RNb2RlKSB7XG4gICAgICBjb25zdCBpbmxpZXJOdW0gPSBNYXRoLm1heCgzLCBNYXRoLmZsb29yKHdvcmxkQ29vcmRzLmxlbmd0aCAqIGlubGllclByb2IpIC0gMSk7XG4gICAgICBmb3IgKGxldCBuID0gMDsgbiA8IHdvcmxkQ29vcmRzLmxlbmd0aDsgbisrKSB7XG4gICAgICAgIEUyW25dID0gRVtuXTtcbiAgICAgIH1cbiAgICAgIEUyLnNvcnQoKGEsIGIpID0+IHtyZXR1cm4gYS1iO30pO1xuXG4gICAgICBLMiA9IE1hdGgubWF4KEUyW2lubGllck51bV0gKiBLMl9GQUNUT1IsIDE2LjApO1xuICAgICAgZm9yIChsZXQgbiA9IDA7IG4gPCB3b3JsZENvb3Jkcy5sZW5ndGg7IG4rKykge1xuICAgICAgICBpZiAoRTJbbl0gPiBLMikgZXJyMSArPSBLMi8gNjtcbiAgICAgICAgZWxzZSBlcnIxICs9ICBLMi82LjAgKiAoMS4wIC0gKDEuMC1FMltuXS9LMikqKDEuMC1FMltuXS9LMikqKDEuMC1FMltuXS9LMikpO1xuICAgICAgfVxuICAgIH0gZWxzZSB7XG4gICAgICBmb3IgKGxldCBuID0gMDsgbiA8IHdvcmxkQ29vcmRzLmxlbmd0aDsgbisrKSB7XG4gICAgICAgIGVycjEgKz0gRVtuXTtcbiAgICAgIH1cbiAgICB9XG4gICAgZXJyMSAvPSB3b3JsZENvb3Jkcy5sZW5ndGg7XG5cbiAgICAvL2NvbnNvbGUubG9nKFwiaWNwIGxvb3BcIiwgaW5saWVyUHJvYiwgbCwgZXJyMSk7XG5cbiAgICBpZiAoZXJyMSA8IElDUF9CUkVBS19MT09QX0VSUk9SX1RIUkVTSCkgYnJlYWs7XG4gICAgLy9pZiAobCA+IDAgJiYgZXJyMSA8IElDUF9CUkVBS19MT09QX0VSUk9SX1RIUkVTSDIgJiYgZXJyMS9lcnIwID4gSUNQX0JSRUFLX0xPT1BfRVJST1JfUkFUSU9fVEhSRVNIKSBicmVhaztcbiAgICBpZiAobCA+IDAgJiYgZXJyMS9lcnIwID4gSUNQX0JSRUFLX0xPT1BfRVJST1JfUkFUSU9fVEhSRVNIKSBicmVhaztcbiAgICBpZiAobCA9PT0gSUNQX01BWF9MT09QKSBicmVhaztcblxuICAgIGVycjAgPSBlcnIxO1xuXG4gICAgY29uc3QgZFUgPSBbXTtcbiAgICBjb25zdCBhbGxKX1VfUyA9IFtdO1xuICAgIGZvciAobGV0IG4gPSAwOyBuIDwgd29ybGRDb29yZHMubGVuZ3RoOyBuKyspIHtcbiAgICAgIGlmIChpc1JvYnVzdE1vZGUgJiYgRVtuXSA+IEsyKSB7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICBjb25zdCBKX1VfUyA9IF9nZXRKX1VfUyh7bW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybSwgbW9kZWxWaWV3VHJhbnNmb3JtLCBwcm9qZWN0aW9uVHJhbnNmb3JtLCB3b3JsZENvb3JkOiB3b3JsZENvb3Jkc1tuXX0pO1xuXG4gICAgICBpZiAoaXNSb2J1c3RNb2RlKSB7XG4gICAgICAgIGNvbnN0IFcgPSAoMS4wIC0gRVtuXS9LMikqKDEuMCAtIEVbbl0vSzIpO1xuXG4gICAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgMjsgaisrKSB7XG4gICAgICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCA2OyBpKyspIHtcbiAgICAgICAgICAgIEpfVV9TW2pdW2ldICo9IFc7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIGRVLnB1c2goW2R4c1tuXSAqIFddKTtcbiAgICAgICAgZFUucHVzaChbZHlzW25dICogV10pO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgZFUucHVzaChbZHhzW25dXSk7XG4gICAgICAgIGRVLnB1c2goW2R5c1tuXV0pO1xuICAgICAgfVxuXG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IEpfVV9TLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgIGFsbEpfVV9TLnB1c2goSl9VX1NbaV0pO1xuICAgICAgfVxuICAgIH1cblxuICAgIGNvbnN0IGRTID0gX2dldERlbHRhUyh7ZFUsIEpfVV9TOiBhbGxKX1VfU30pO1xuICAgIGlmIChkUyA9PT0gbnVsbCkgYnJlYWs7XG5cbiAgICBtb2RlbFZpZXdUcmFuc2Zvcm0gPSBfdXBkYXRlTW9kZWxWaWV3VHJhbnNmb3JtKHttb2RlbFZpZXdUcmFuc2Zvcm0sIGRTfSk7XG4gIH1cbiAgcmV0dXJuIHttb2RlbFZpZXdUcmFuc2Zvcm0sIGVycjogZXJyMX07XG59XG5cbmNvbnN0IF91cGRhdGVNb2RlbFZpZXdUcmFuc2Zvcm0gPSAoe21vZGVsVmlld1RyYW5zZm9ybSwgZFN9KSA9PiB7XG4gIC8qKlxuICAgKiBkUyBoYXMgNiBwYXJhZ3JhbXMsIGZpcnN0IGhhbGYgaXMgcm90YXRpb24sIHNlY29uZCBoYWxmIGlzIHRyYW5zbGF0aW9uXG4gICAqIHJvdGF0aW9uIGlzIGV4cHJlc3NlZCBpbiBhbmdsZS1heGlzLCBcbiAgICogICBbU1swXSwgU1sxXSAsU1syXV0gaXMgdGhlIGF4aXMgb2Ygcm90YXRpb24sIGFuZCB0aGUgbWFnbml0dWRlIGlzIHRoZSBhbmdsZVxuICAgKi9cbiAgbGV0IHJhID0gZFNbMF0gKiBkU1swXSArIGRTWzFdICogZFNbMV0gKyBkU1syXSAqIGRTWzJdO1xuICBsZXQgcTAsIHExLCBxMjtcbiAgaWYoIHJhIDwgMC4wMDAwMDEgKSB7XG4gICAgcTAgPSAxLjA7XG4gICAgcTEgPSAwLjA7XG4gICAgcTIgPSAwLjA7XG4gICAgcmEgPSAwLjA7XG4gIH0gZWxzZSB7XG4gICAgcmEgPSBNYXRoLnNxcnQocmEpO1xuICAgIHEwID0gZFNbMF0gLyByYTtcbiAgICBxMSA9IGRTWzFdIC8gcmE7XG4gICAgcTIgPSBkU1syXSAvIHJhO1xuICB9XG5cbiAgY29uc3QgY3JhID0gTWF0aC5jb3MocmEpO1xuICBjb25zdCBzcmEgPSBNYXRoLnNpbihyYSk7XG4gIGNvbnN0IG9uZV9jcmEgPSAxLjAgLSBjcmE7XG5cbiAgLy8gbWF0IGlzIFtSfHRdLCAzRCByb3RhdGlvbiBhbmQgdHJhbnNsYXRpb25cbiAgbWF0WzBdWzBdID0gcTAqcTAqb25lX2NyYSArIGNyYTtcbiAgbWF0WzBdWzFdID0gcTAqcTEqb25lX2NyYSAtIHEyKnNyYTtcbiAgbWF0WzBdWzJdID0gcTAqcTIqb25lX2NyYSArIHExKnNyYTtcbiAgbWF0WzBdWzNdID0gZFNbM107XG4gIG1hdFsxXVswXSA9IHExKnEwKm9uZV9jcmEgKyBxMipzcmE7XG4gIG1hdFsxXVsxXSA9IHExKnExKm9uZV9jcmEgKyBjcmE7XG4gIG1hdFsxXVsyXSA9IHExKnEyKm9uZV9jcmEgLSBxMCpzcmE7XG4gIG1hdFsxXVszXSA9IGRTWzRdXG4gIG1hdFsyXVswXSA9IHEyKnEwKm9uZV9jcmEgLSBxMSpzcmE7XG4gIG1hdFsyXVsxXSA9IHEyKnExKm9uZV9jcmEgKyBxMCpzcmE7XG4gIG1hdFsyXVsyXSA9IHEyKnEyKm9uZV9jcmEgKyBjcmE7XG4gIG1hdFsyXVszXSA9IGRTWzVdO1xuXG4gIC8vIHRoZSB1cGRhdGVkIHRyYW5zZm9ybSBpcyB0aGUgb3JpZ2luYWwgdHJhbnNmb3JtIHggZGVsdGEgdHJhbnNmb3JtXG4gIGNvbnN0IG1hdDIgPSBbW10sW10sW11dO1xuICBmb3IgKGxldCBqID0gMDsgaiA8IDM7IGorKyApIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IDQ7IGkrKyApIHtcbiAgICAgIG1hdDJbal1baV0gPSBtb2RlbFZpZXdUcmFuc2Zvcm1bal1bMF0gKiBtYXRbMF1baV1cbiAgICAgICAgICAgICAgICAgICArIG1vZGVsVmlld1RyYW5zZm9ybVtqXVsxXSAqIG1hdFsxXVtpXVxuICAgICAgICAgICAgICAgICAgICsgbW9kZWxWaWV3VHJhbnNmb3JtW2pdWzJdICogbWF0WzJdW2ldO1xuICAgIH1cbiAgICBtYXQyW2pdWzNdICs9IG1vZGVsVmlld1RyYW5zZm9ybVtqXVszXTtcbiAgfVxuICByZXR1cm4gbWF0Mjtcbn1cblxuY29uc3QgX2dldERlbHRhUyA9ICh7ZFUsIEpfVV9TfSkgPT4ge1xuICBjb25zdCBKID0gbmV3IE1hdHJpeChKX1VfUyk7XG4gIGNvbnN0IFUgPSBuZXcgTWF0cml4KGRVKTtcblxuICBjb25zdCBKVCA9IEoudHJhbnNwb3NlKCk7XG4gIGNvbnN0IEpUSiA9IEpULm1tdWwoSik7XG4gIGNvbnN0IEpUVSA9IEpULm1tdWwoVSk7XG5cbiAgbGV0IEpUSkludjtcbiAgdHJ5IHtcbiAgICBKVEpJbnYgPSBpbnZlcnNlKEpUSik7XG4gIH0gY2F0Y2ggKGUpIHtcbiAgICByZXR1cm4gbnVsbDtcbiAgfVxuXG4gIGNvbnN0IFMgPSBKVEpJbnYubW11bChKVFUpO1xuICByZXR1cm4gUy50bzFEQXJyYXkoKTtcbn1cblxuY29uc3QgX2dldEpfVV9TID0gKHttb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtLCBtb2RlbFZpZXdUcmFuc2Zvcm0sIHByb2plY3Rpb25UcmFuc2Zvcm0sIHdvcmxkQ29vcmR9KSA9PiB7XG4gIGNvbnN0IFQgPSBtb2RlbFZpZXdUcmFuc2Zvcm07XG4gIGNvbnN0IHt4LCB5LCB6fSA9IHdvcmxkQ29vcmQ7XG5cbiAgY29uc3QgdSA9IGFwcGx5TW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybShtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtLCB4LCB5LCB6KTtcblxuICBjb25zdCB6MiA9IHUueiAqIHUuejtcbiAgLy8gUXVlc3Rpb246IFRoaXMgaXMgdGhlIG1vc3QgY29uZnVzaW5nIG1hdHJpeCB0byBtZS4gSSd2ZSBubyBpZGVhIGhvdyB0byBkZXJpdmUgdGhpcy5cbiAgLy9KX1VfWGNbMF1bMF0gPSAocHJvamVjdGlvblRyYW5zZm9ybVswXVswXSAqIHUueiAtIHByb2plY3Rpb25UcmFuc2Zvcm1bMl1bMF0gKiB1LngpIC8gejI7XG4gIC8vSl9VX1hjWzBdWzFdID0gKHByb2plY3Rpb25UcmFuc2Zvcm1bMF1bMV0gKiB1LnogLSBwcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzFdICogdS54KSAvIHoyO1xuICAvL0pfVV9YY1swXVsyXSA9IChwcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzJdICogdS56IC0gcHJvamVjdGlvblRyYW5zZm9ybVsyXVsyXSAqIHUueCkgLyB6MjtcbiAgLy9KX1VfWGNbMV1bMF0gPSAocHJvamVjdGlvblRyYW5zZm9ybVsxXVswXSAqIHUueiAtIHByb2plY3Rpb25UcmFuc2Zvcm1bMl1bMF0gKiB1LnkpIC8gejI7XG4gIC8vSl9VX1hjWzFdWzFdID0gKHByb2plY3Rpb25UcmFuc2Zvcm1bMV1bMV0gKiB1LnogLSBwcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzFdICogdS55KSAvIHoyO1xuICAvL0pfVV9YY1sxXVsyXSA9IChwcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzJdICogdS56IC0gcHJvamVjdGlvblRyYW5zZm9ybVsyXVsyXSAqIHUueSkgLyB6MjtcbiAgXG4gIC8vIFRoZSBhYm92ZSBpcyB0aGUgb3JpZ2luYWwgaW1wbGVtZW50YXRpb24sIGJ1dCBzaW1wbGlmeSB0byBiZWxvdyBiZWN1YXNlIHByb2pldGlvblRyYW5zZm9ybVsyXVswXSBhbmQgWzJdWzFdIGFyZSB6ZXJvXG4gIEpfVV9YY1swXVswXSA9IChwcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzBdICogdS56KSAvIHoyO1xuICBKX1VfWGNbMF1bMV0gPSAocHJvamVjdGlvblRyYW5zZm9ybVswXVsxXSAqIHUueikgLyB6MjtcbiAgSl9VX1hjWzBdWzJdID0gKHByb2plY3Rpb25UcmFuc2Zvcm1bMF1bMl0gKiB1LnogLSBwcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzJdICogdS54KSAvIHoyO1xuICBKX1VfWGNbMV1bMF0gPSAocHJvamVjdGlvblRyYW5zZm9ybVsxXVswXSAqIHUueikgLyB6MjtcbiAgSl9VX1hjWzFdWzFdID0gKHByb2plY3Rpb25UcmFuc2Zvcm1bMV1bMV0gKiB1LnopIC8gejI7XG4gIEpfVV9YY1sxXVsyXSA9IChwcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzJdICogdS56IC0gcHJvamVjdGlvblRyYW5zZm9ybVsyXVsyXSAqIHUueSkgLyB6MjtcblxuICAvKlxuICAgIEpfWGNfUyBzaG91bGQgYmUgbGlrZSB0aGlzLCBidXQgeiBpcyB6ZXJvLCBzbyB3ZSBjYW4gc2ltcGxpZnlcbiAgICBbVFswXVsyXSAqIHkgLSBUWzBdWzFdICogeiwgVFswXVswXSAqIHogLSBUWzBdWzJdICogeCwgVFswXVsxXSAqIHggLSBUWzBdWzBdICogeSwgVFswXVswXSwgVFswXVsxXSwgVFswXVsyXV0sXG4gICAgW1RbMV1bMl0gKiB5IC0gVFsxXVsxXSAqIHosIFRbMV1bMF0gKiB6IC0gVFsxXVsyXSAqIHgsIFRbMV1bMV0gKiB4IC0gVFsxXVswXSAqIHksIFRbMV1bMF0sIFRbMV1bMV0sIFRbMV1bMl1dLFxuICAgIFtUWzJdWzJdICogeSAtIFRbMl1bMV0gKiB6LCBUWzJdWzBdICogeiAtIFRbMl1bMl0gKiB4LCBUWzJdWzFdICogeCAtIFRbMl1bMF0gKiB5LCBUWzJdWzBdLCBUWzJdWzFdLCBUWzJdWzJdXSxcbiAgKi9cbiAgSl9YY19TWzBdWzBdID0gVFswXVsyXSAqIHk7XG4gIEpfWGNfU1swXVsxXSA9IC1UWzBdWzJdICogeDtcbiAgSl9YY19TWzBdWzJdID0gVFswXVsxXSAqIHggLSBUWzBdWzBdICogeTtcbiAgSl9YY19TWzBdWzNdID0gVFswXVswXTtcbiAgSl9YY19TWzBdWzRdID0gVFswXVsxXTsgXG4gIEpfWGNfU1swXVs1XSA9IFRbMF1bMl07XG5cbiAgSl9YY19TWzFdWzBdID0gVFsxXVsyXSAqIHk7XG4gIEpfWGNfU1sxXVsxXSA9IC1UWzFdWzJdICogeDtcbiAgSl9YY19TWzFdWzJdID0gVFsxXVsxXSAqIHggLSBUWzFdWzBdICogeTtcbiAgSl9YY19TWzFdWzNdID0gVFsxXVswXTtcbiAgSl9YY19TWzFdWzRdID0gVFsxXVsxXTtcbiAgSl9YY19TWzFdWzVdID0gVFsxXVsyXTtcblxuICBKX1hjX1NbMl1bMF0gPSBUWzJdWzJdICogeTtcbiAgSl9YY19TWzJdWzFdID0gLVRbMl1bMl0gKiB4O1xuICBKX1hjX1NbMl1bMl0gPSBUWzJdWzFdICogeCAtIFRbMl1bMF0gKiB5O1xuICBKX1hjX1NbMl1bM10gPSBUWzJdWzBdO1xuICBKX1hjX1NbMl1bNF0gPSBUWzJdWzFdO1xuICBKX1hjX1NbMl1bNV0gPSBUWzJdWzJdO1xuXG4gIGNvbnN0IEpfVV9TID0gW1tdLCBbXV07XG4gIGZvciAobGV0IGogPSAwOyBqIDwgMjsgaisrKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCA2OyBpKyspIHtcbiAgICAgIEpfVV9TW2pdW2ldID0gMC4wO1xuICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCAzOyBrKysgKSB7XG4gICAgICAgIEpfVV9TW2pdW2ldICs9IEpfVV9YY1tqXVtrXSAqIEpfWGNfU1trXVtpXTtcbiAgICAgIH1cbiAgICB9XG4gIH1cbiAgcmV0dXJuIEpfVV9TO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IHtcbiAgcmVmaW5lRXN0aW1hdGVcbn1cbiIsImNvbnN0IGJ1aWxkTW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybSA9IChwcm9qZWN0aW9uVHJhbnNmb3JtLCBtb2RlbFZpZXdUcmFuc2Zvcm0pID0+IHtcbiAgLy8gYXNzdW1lIHRoZSBwcm9qZWN0VHJhbnNmb3JtIGhhcyB0aGUgZm9sbG93aW5nIGZvcm1hdDpcbiAgLy8gW1tmeCwgMCwgY3hdLFxuICAvLyAgWzAsIGZ5LCBjeV1cbiAgLy8gIFswLCAwLCAxXV1cbiAgY29uc3QgbW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybSA9IFtcbiAgICBbXG4gICAgICBwcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzBdICogbW9kZWxWaWV3VHJhbnNmb3JtWzBdWzBdICsgcHJvamVjdGlvblRyYW5zZm9ybVswXVsyXSAqIG1vZGVsVmlld1RyYW5zZm9ybVsyXVswXSxcbiAgICAgIHByb2plY3Rpb25UcmFuc2Zvcm1bMF1bMF0gKiBtb2RlbFZpZXdUcmFuc2Zvcm1bMF1bMV0gKyBwcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzJdICogbW9kZWxWaWV3VHJhbnNmb3JtWzJdWzFdLFxuICAgICAgcHJvamVjdGlvblRyYW5zZm9ybVswXVswXSAqIG1vZGVsVmlld1RyYW5zZm9ybVswXVsyXSArIHByb2plY3Rpb25UcmFuc2Zvcm1bMF1bMl0gKiBtb2RlbFZpZXdUcmFuc2Zvcm1bMl1bMl0sXG4gICAgICBwcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzBdICogbW9kZWxWaWV3VHJhbnNmb3JtWzBdWzNdICsgcHJvamVjdGlvblRyYW5zZm9ybVswXVsyXSAqIG1vZGVsVmlld1RyYW5zZm9ybVsyXVszXSxcbiAgICBdLFxuICAgIFtcbiAgICAgIHByb2plY3Rpb25UcmFuc2Zvcm1bMV1bMV0gKiBtb2RlbFZpZXdUcmFuc2Zvcm1bMV1bMF0gKyBwcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzJdICogbW9kZWxWaWV3VHJhbnNmb3JtWzJdWzBdLFxuICAgICAgcHJvamVjdGlvblRyYW5zZm9ybVsxXVsxXSAqIG1vZGVsVmlld1RyYW5zZm9ybVsxXVsxXSArIHByb2plY3Rpb25UcmFuc2Zvcm1bMV1bMl0gKiBtb2RlbFZpZXdUcmFuc2Zvcm1bMl1bMV0sXG4gICAgICBwcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzFdICogbW9kZWxWaWV3VHJhbnNmb3JtWzFdWzJdICsgcHJvamVjdGlvblRyYW5zZm9ybVsxXVsyXSAqIG1vZGVsVmlld1RyYW5zZm9ybVsyXVsyXSxcbiAgICAgIHByb2plY3Rpb25UcmFuc2Zvcm1bMV1bMV0gKiBtb2RlbFZpZXdUcmFuc2Zvcm1bMV1bM10gKyBwcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzJdICogbW9kZWxWaWV3VHJhbnNmb3JtWzJdWzNdLFxuICAgIF0sXG4gICAgW1xuICAgICAgbW9kZWxWaWV3VHJhbnNmb3JtWzJdWzBdLFxuICAgICAgbW9kZWxWaWV3VHJhbnNmb3JtWzJdWzFdLFxuICAgICAgbW9kZWxWaWV3VHJhbnNmb3JtWzJdWzJdLFxuICAgICAgbW9kZWxWaWV3VHJhbnNmb3JtWzJdWzNdLFxuICAgIF1cbiAgXTtcbiAgcmV0dXJuIG1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm07XG4gIFxuICAvKlxuICAvLyB0aGlzIGlzIHRoZSBmdWxsIGNvbXB1dGF0aW9uIGlmIHRoZSBwcm9qZWN0VHJhbnNmb3JtIGRvZXMgbm90IGxvb2sgbGlrZSB0aGUgZXhwZWN0ZWQgZm9ybWF0LCBidXQgbW9yZSBjb21wdXRhdGlvbnNcbiAgLy8gIFxuICBjb25zdCBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtID0gW1tdLFtdLFtdXTtcbiAgZm9yIChsZXQgaiA9IDA7IGogPCAzOyBqKysgKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCA0OyBpKyspIHtcbiAgICAgIG1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm1bal1baV0gPSBwcm9qZWN0aW9uVHJhbnNmb3JtW2pdWzBdICogbW9kZWxWaWV3VHJhbnNmb3JtWzBdW2ldXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICsgcHJvamVjdGlvblRyYW5zZm9ybVtqXVsxXSAqIG1vZGVsVmlld1RyYW5zZm9ybVsxXVtpXVxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICArIHByb2plY3Rpb25UcmFuc2Zvcm1bal1bMl0gKiBtb2RlbFZpZXdUcmFuc2Zvcm1bMl1baV07XG4gICAgfVxuICB9XG4gIHJldHVybiBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtO1xuICAqL1xufVxuXG5jb25zdCBhcHBseU1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm0gPSAobW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybSwgeCwgeSwgeikgPT4ge1xuICAvLyBhc3N1bWUgeiBpcyB6ZXJvXG4gIGNvbnN0IHV4ID0gbW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybVswXVswXSAqIHggKyBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzFdICogeSArIG1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm1bMF1bM107XG4gIGNvbnN0IHV5ID0gbW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybVsxXVswXSAqIHggKyBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzFdICogeSArIG1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm1bMV1bM107XG4gIGNvbnN0IHV6ID0gbW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybVsyXVswXSAqIHggKyBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzFdICogeSArIG1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm1bMl1bM107XG4gIHJldHVybiB7eDogdXgsIHk6IHV5LCB6OiB1en07XG59XG5cbmNvbnN0IGNvbXB1dGVTY3JlZW5Db29yZGlhdGUgPSAobW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybSwgeCwgeSwgeikgPT4ge1xuICBjb25zdCB7eDogdXgsIHk6IHV5LCB6OiB1en0gPSBhcHBseU1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm0obW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybSwgeCwgeSwgeik7XG4gIC8vaWYoIE1hdGguYWJzKHV6KSA8IDAuMDAwMDAxICkgcmV0dXJuIG51bGw7XG4gIHJldHVybiB7eDogdXgvdXosIHk6IHV5L3V6fTtcbn1cblxuY29uc3Qgc2NyZWVuVG9NYXJrZXJDb29yZGluYXRlID0gKG1vZGVsVmlld1Byb2plY3Rpb25UcmFuc2Zvcm0sIHN4LCBzeSkgPT4ge1xuICBjb25zdCBjMTEgPSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzBdICogc3ggLSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzBdO1xuICBjb25zdCBjMTIgPSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzFdICogc3ggLSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzFdO1xuICBjb25zdCBjMjEgPSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzBdICogc3kgLSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzBdO1xuICBjb25zdCBjMjIgPSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzJdWzFdICogc3kgLSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzFdO1xuICBjb25zdCBiMSAgPSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzBdWzNdIC0gbW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybVsyXVszXSAqIHN4O1xuICBjb25zdCBiMiAgPSBtb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtWzFdWzNdIC0gbW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybVsyXVszXSAqIHN5O1xuXG4gIGNvbnN0IG0gPSBjMTEgKiBjMjIgLSBjMTIgKiBjMjE7XG4gIHJldHVybiB7XG4gICAgeDogKGMyMiAqIGIxIC0gYzEyICogYjIpIC8gbSxcbiAgICB5OiAoYzExICogYjIgLSBjMjEgKiBiMSkgLyBtXG4gIH1cbn1cblxubW9kdWxlLmV4cG9ydHMgPSB7XG4gIGJ1aWxkTW9kZWxWaWV3UHJvamVjdGlvblRyYW5zZm9ybSxcbiAgYXBwbHlNb2RlbFZpZXdQcm9qZWN0aW9uVHJhbnNmb3JtLFxuICBjb21wdXRlU2NyZWVuQ29vcmRpYXRlLFxufVxuIiwiLy8gRmFzdCBjb21wdXRhdGlvbiBvbiBudW1iZXIgb2YgYml0IHNldHNcbi8vIFJlZjogaHR0cHM6Ly9ncmFwaGljcy5zdGFuZm9yZC5lZHUvfnNlYW5kZXIvYml0aGFja3MuaHRtbCNDb3VudEJpdHNTZXRQYXJhbGxlbFxuY29uc3QgY29tcHV0ZSA9IChvcHRpb25zKSA9PiB7XG4gIGNvbnN0IHt2MSwgdjJ9ID0gb3B0aW9ucztcbiAgbGV0IGQgPSAwO1xuXG4gIGZvciAobGV0IGkgPSAwOyBpIDwgdjEubGVuZ3RoOyBpKyspIHtcbiAgICBsZXQgeCA9ICh2MVtpXSBeIHYyW2ldKSA+Pj4gMDtcbiAgICBkICs9IGJpdENvdW50KHgpO1xuICB9XG4gIHJldHVybiBkO1xufVxuXG5jb25zdCBiaXRDb3VudCA9ICh2KSA9PiB7XG4gIHZhciBjID0gdiAtICgodiA+PiAxKSAmIDB4NTU1NTU1NTUpO1xuICBjID0gKChjID4+IDIpICYgMHgzMzMzMzMzMykgKyAoYyAmIDB4MzMzMzMzMzMpO1xuICBjID0gKChjID4+IDQpICsgYykgJiAweDBGMEYwRjBGO1xuICBjID0gKChjID4+IDgpICsgYykgJiAweDAwRkYwMEZGO1xuICBjID0gKChjID4+IDE2KSArIGMpICYgMHgwMDAwRkZGRjtcbiAgcmV0dXJuIGM7XG59XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuICBjb21wdXRlXG59O1xuIiwiY29uc3Qga0hvdWdoQmluRGVsdGEgPSAxO1xuXG4vLyBtYXRoY2VzIFtxdWVyeXBvaW50SW5kZXg6eCwga2V5cG9pbnRJbmRleDogeF1cbmNvbnN0IGNvbXB1dGVIb3VnaE1hdGNoZXMgPSAob3B0aW9ucykgPT4ge1xuICBjb25zdCB7a2V5d2lkdGgsIGtleWhlaWdodCwgcXVlcnl3aWR0aCwgcXVlcnloZWlnaHQsIG1hdGNoZXN9ID0gb3B0aW9ucztcblxuICBjb25zdCBtYXhYID0gcXVlcnl3aWR0aCAqIDEuMjtcbiAgY29uc3QgbWluWCA9IC1tYXhYO1xuICBjb25zdCBtYXhZID0gcXVlcnloZWlnaHQgKiAxLjI7XG4gIGNvbnN0IG1pblkgPSAtbWF4WTtcbiAgY29uc3QgbnVtQW5nbGVCaW5zID0gMTI7XG4gIGNvbnN0IG51bVNjYWxlQmlucyA9IDEwO1xuICBjb25zdCBtaW5TY2FsZSA9IC0xO1xuICBjb25zdCBtYXhTY2FsZSA9IDE7XG4gIGNvbnN0IHNjYWxlSyA9IDEwLjA7XG4gIGNvbnN0IHNjYWxlT25lT3ZlckxvZ0sgPSAxLjAgLyBNYXRoLmxvZyhzY2FsZUspO1xuICBjb25zdCBtYXhEaW0gPSBNYXRoLm1heChrZXl3aWR0aCwga2V5aGVpZ2h0KTtcbiAgY29uc3Qga2V5Y2VudGVyWCA9IE1hdGguZmxvb3Ioa2V5d2lkdGggLyAyKTtcbiAgY29uc3Qga2V5Y2VudGVyWSA9IE1hdGguZmxvb3Ioa2V5aGVpZ2h0IC8gMik7XG5cbiAgLy8gY29tcHV0ZSBudW1YQmlucyBhbmQgbnVtWUJpbnMgYmFzZWQgb24gbWF0Y2hlc1xuICBjb25zdCBwcm9qZWN0ZWREaW1zID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbWF0Y2hlcy5sZW5ndGg7IGkrKykge1xuICAgIGNvbnN0IHF1ZXJ5c2NhbGUgPSBtYXRjaGVzW2ldLnF1ZXJ5cG9pbnQuc2NhbGU7XG4gICAgY29uc3Qga2V5c2NhbGUgPSBtYXRjaGVzW2ldLmtleXBvaW50LnNjYWxlO1xuICAgIGlmIChrZXlzY2FsZSA9PSAwKSBjb25zb2xlLmxvZyhcIkVSUk9SIGRpdmlkZSB6ZXJvXCIpO1xuICAgIGNvbnN0IHNjYWxlID0gcXVlcnlzY2FsZSAvIGtleXNjYWxlO1xuICAgIHByb2plY3RlZERpbXMucHVzaCggc2NhbGUgKiBtYXhEaW0gKTtcbiAgfVxuXG4gIC8vIFRPRE8gb3B0aW1pemUgbWVkaWFuXG4gIC8vICAgd2VpcmQuIG1lZGlhbiBzaG91bGQgYmUgW01hdGguZmxvb3IocHJvamVjdGVkRGltcy5sZW5ndGgvMikgLSAxXSA/XG4gIHByb2plY3RlZERpbXMuc29ydCgoYTEsIGEyKSA9PiB7cmV0dXJuIGExIC0gYTJ9KTtcbiAgY29uc3QgbWVkaWFuUHJvamVjdGVkRGltID0gcHJvamVjdGVkRGltc1sgTWF0aC5mbG9vcihwcm9qZWN0ZWREaW1zLmxlbmd0aC8yKSAtIChwcm9qZWN0ZWREaW1zLmxlbmd0aCUyPT0wPzE6MCkgLTEgXTtcblxuICBjb25zdCBiaW5TaXplID0gMC4yNSAqIG1lZGlhblByb2plY3RlZERpbTtcbiAgY29uc3QgbnVtWEJpbnMgPSBNYXRoLm1heCg1LCBNYXRoLmNlaWwoKG1heFggLSBtaW5YKSAvIGJpblNpemUpKTtcbiAgY29uc3QgbnVtWUJpbnMgPSBNYXRoLm1heCg1LCBNYXRoLmNlaWwoKG1heFkgLSBtaW5ZKSAvIGJpblNpemUpKTtcblxuICBjb25zdCBudW1YWUJpbnMgPSBudW1YQmlucyAqIG51bVlCaW5zO1xuICBjb25zdCBudW1YWUFuZ2xlQmlucyA9IG51bVhZQmlucyAqIG51bUFuZ2xlQmlucztcblxuICAvLyBkbyB2b3RpbmdcbiAgY29uc3QgcXVlcnlwb2ludFZhbGlkcyA9IFtdO1xuICBjb25zdCBxdWVyeXBvaW50QmluTG9jYXRpb25zID0gW107XG4gIGNvbnN0IHZvdGVzID0ge307XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbWF0Y2hlcy5sZW5ndGg7IGkrKykge1xuICAgIGNvbnN0IHF1ZXJ5cG9pbnQgPSBtYXRjaGVzW2ldLnF1ZXJ5cG9pbnQ7XG4gICAgY29uc3Qga2V5cG9pbnQgPSBtYXRjaGVzW2ldLmtleXBvaW50O1xuXG4gICAgY29uc3Qge3gsIHksIHNjYWxlLCBhbmdsZX0gPSBfbWFwQ29ycmVzcG9uZGVuY2Uoe3F1ZXJ5cG9pbnQsIGtleXBvaW50LCBrZXljZW50ZXJYLCBrZXljZW50ZXJZLCBzY2FsZU9uZU92ZXJMb2dLfSk7XG5cbiAgICAvLyBDaGVjayB0aGF0IHRoZSB2b3RlIGlzIHdpdGhpbiByYW5nZVxuICAgIGlmICh4IDwgbWluWCB8fCB4ID49IG1heFggfHwgeSA8IG1pblkgfHwgeSA+PSBtYXhZIHx8IGFuZ2xlIDw9IC1NYXRoLlBJIHx8IGFuZ2xlID4gTWF0aC5QSSB8fCBzY2FsZSA8IG1pblNjYWxlIHx8IHNjYWxlID49IG1heFNjYWxlKSB7XG4gICAgICBxdWVyeXBvaW50VmFsaWRzW2ldID0gZmFsc2U7XG4gICAgICBjb250aW51ZTtcbiAgICB9XG5cbiAgICAvLyBtYXAgcHJvcGVydGllcyB0byBiaW5zXG4gICAgbGV0IGZiaW5YID0gbnVtWEJpbnMgKiAoeCAtIG1pblgpIC8gKG1heFggLSBtaW5YKTtcbiAgICBsZXQgZmJpblkgPSBudW1ZQmlucyAqICh5IC0gbWluWSkgLyAobWF4WSAtIG1pblkpO1xuICAgIGxldCBmYmluQW5nbGUgPSBudW1BbmdsZUJpbnMgKiAoYW5nbGUgKyBNYXRoLlBJKSAvICgyLjAgKiBNYXRoLlBJKTtcbiAgICBsZXQgZmJpblNjYWxlID0gbnVtU2NhbGVCaW5zICogKHNjYWxlIC0gbWluU2NhbGUpIC8gKG1heFNjYWxlIC0gbWluU2NhbGUpO1xuXG4gICAgcXVlcnlwb2ludEJpbkxvY2F0aW9uc1tpXSA9IHtiaW5YOiBmYmluWCwgYmluWTogZmJpblksIGJpbkFuZ2xlOiBmYmluQW5nbGUsIGJpblNjYWxlOiBmYmluU2NhbGV9O1xuXG4gICAgbGV0IGJpblggPSBNYXRoLmZsb29yKGZiaW5YIC0gMC41KTtcbiAgICBsZXQgYmluWSA9IE1hdGguZmxvb3IoZmJpblkgLSAwLjUpO1xuICAgIGxldCBiaW5TY2FsZSA9IE1hdGguZmxvb3IoZmJpblNjYWxlIC0gMC41KTtcbiAgICBsZXQgYmluQW5nbGUgPSAoTWF0aC5mbG9vcihmYmluQW5nbGUgLSAwLjUpICsgbnVtQW5nbGVCaW5zKSAlIG51bUFuZ2xlQmlucztcblxuICAgIC8vIGNoZWNrIGNhbiB2b3RlIGFsbCAxNiBiaW5zXG4gICAgaWYgKGJpblggPCAwIHx8IGJpblggKyAxID49IG51bVhCaW5zIHx8IGJpblkgPCAwIHx8IGJpblkgKyAxID49IG51bVlCaW5zIHx8IGJpblNjYWxlIDwgMCB8fCBiaW5TY2FsZSArMSA+PSBudW1TY2FsZUJpbnMpIHtcbiAgICAgIHF1ZXJ5cG9pbnRWYWxpZHNbaV0gPSBmYWxzZTtcbiAgICAgIGNvbnRpbnVlO1xuICAgIH1cblxuICAgIGZvciAobGV0IGR4ID0gMDsgZHggPCAyOyBkeCsrKSB7XG4gICAgICBsZXQgYmluWDIgPSBiaW5YICsgZHg7XG5cbiAgICAgIGZvciAobGV0IGR5ID0gMDsgZHkgPCAyOyBkeSsrKSB7XG4gICAgICAgIGxldCBiaW5ZMiA9IGJpblkgKyBkeTtcblxuICAgICAgICBmb3IgKGxldCBkYW5nbGUgPSAwOyBkYW5nbGUgPCAyOyBkYW5nbGUrKykge1xuICAgICAgICAgIGxldCBiaW5BbmdsZTIgPSAoYmluQW5nbGUgKyBkYW5nbGUpICUgbnVtQW5nbGVCaW5zO1xuXG4gICAgICAgICAgZm9yIChsZXQgZHNjYWxlID0gMDsgZHNjYWxlIDwgMjsgZHNjYWxlKyspIHtcbiAgICAgICAgICAgIGxldCBiaW5TY2FsZTIgPSBiaW5TY2FsZSArIGRzY2FsZTtcblxuICAgICAgICAgICAgY29uc3QgYmluSW5kZXggPSBiaW5YMiArIGJpblkyICogbnVtWEJpbnMgKyBiaW5BbmdsZTIgKiBudW1YWUJpbnMgKyBiaW5TY2FsZTIgKiBudW1YWUFuZ2xlQmlucztcblxuICAgICAgICAgICAgaWYgKHZvdGVzW2JpbkluZGV4XSA9PT0gdW5kZWZpbmVkKSB2b3Rlc1tiaW5JbmRleF0gPSAwO1xuICAgICAgICAgICAgdm90ZXNbYmluSW5kZXhdICs9IDE7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICAgIHF1ZXJ5cG9pbnRWYWxpZHNbaV0gPSB0cnVlO1xuICB9XG5cbiAgbGV0IG1heFZvdGVzID0gMDtcbiAgbGV0IG1heFZvdGVJbmRleCA9IC0xO1xuICBPYmplY3Qua2V5cyh2b3RlcykuZm9yRWFjaCgoaW5kZXgpID0+IHtcbiAgICBpZiAodm90ZXNbaW5kZXhdID4gbWF4Vm90ZXMpIHtcbiAgICAgIG1heFZvdGVzID0gdm90ZXNbaW5kZXhdO1xuICAgICAgbWF4Vm90ZUluZGV4ID0gaW5kZXg7XG4gICAgfVxuICB9KTtcblxuICBpZiAobWF4Vm90ZXMgPCAzKSByZXR1cm4gW107XG5cbiAgLy8gZ2V0IGJhY2sgYmlucyBmcm9tIHZvdGUgaW5kZXhcbiAgY29uc3QgYmluWCA9IE1hdGguZmxvb3IoKChtYXhWb3RlSW5kZXggJSBudW1YWUFuZ2xlQmlucykgJSBudW1YWUJpbnMpICUgbnVtWEJpbnMpO1xuICBjb25zdCBiaW5ZID0gTWF0aC5mbG9vcigoKChtYXhWb3RlSW5kZXggLSBiaW5YKSAlIG51bVhZQW5nbGVCaW5zKSAlIG51bVhZQmlucykgLyBudW1YQmlucyk7XG4gIGNvbnN0IGJpbkFuZ2xlID0gTWF0aC5mbG9vcigoKG1heFZvdGVJbmRleCAtIGJpblggLSAoYmluWSAqIG51bVhCaW5zKSkgJSBudW1YWUFuZ2xlQmlucykgLyBudW1YWUJpbnMpO1xuICBjb25zdCBiaW5TY2FsZSA9IE1hdGguZmxvb3IoKG1heFZvdGVJbmRleCAtIGJpblggLSAoYmluWSAqIG51bVhCaW5zKSAtIChiaW5BbmdsZSAqIG51bVhZQmlucykpIC8gbnVtWFlBbmdsZUJpbnMpO1xuXG4gIC8vY29uc29sZS5sb2coXCJob3VnaCB2b3RlZDogXCIsIHtiaW5YLCBiaW5ZLCBiaW5BbmdsZSwgYmluU2NhbGUsIG1heFZvdGVJbmRleH0pO1xuXG4gIGNvbnN0IGhvdWdoTWF0Y2hlcyA9IFtdO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IG1hdGNoZXMubGVuZ3RoOyBpKyspIHtcbiAgICBpZiAoIXF1ZXJ5cG9pbnRWYWxpZHNbaV0pIGNvbnRpbnVlO1xuXG4gICAgY29uc3QgcXVlcnlCaW5zID0gcXVlcnlwb2ludEJpbkxvY2F0aW9uc1tpXTtcbiAgICAvLyBjb21wdXRlIGJpbiBkaWZmZXJlbmNlXG4gICAgY29uc3QgZGlzdEJpblggPSBNYXRoLmFicyhxdWVyeUJpbnMuYmluWCAtIChiaW5YKzAuNSkpO1xuICAgIGlmIChkaXN0QmluWCA+PSBrSG91Z2hCaW5EZWx0YSkgY29udGludWU7XG5cbiAgICBjb25zdCBkaXN0QmluWSA9IE1hdGguYWJzKHF1ZXJ5Qmlucy5iaW5ZIC0gKGJpblkrMC41KSk7XG4gICAgaWYgKGRpc3RCaW5ZID49IGtIb3VnaEJpbkRlbHRhKSBjb250aW51ZTtcblxuICAgIGNvbnN0IGRpc3RCaW5TY2FsZSA9IE1hdGguYWJzKHF1ZXJ5Qmlucy5iaW5TY2FsZSAtIChiaW5TY2FsZSswLjUpKTtcbiAgICBpZiAoZGlzdEJpblNjYWxlID49IGtIb3VnaEJpbkRlbHRhKSBjb250aW51ZTtcblxuICAgIGNvbnN0IHRlbXAgPSBNYXRoLmFicyhxdWVyeUJpbnMuYmluQW5nbGUgLSAoYmluQW5nbGUrMC41KSk7XG4gICAgY29uc3QgZGlzdEJpbkFuZ2xlID0gTWF0aC5taW4odGVtcCwgbnVtQW5nbGVCaW5zIC0gdGVtcCk7XG4gICAgaWYgKGRpc3RCaW5BbmdsZSA+PSBrSG91Z2hCaW5EZWx0YSkgY29udGludWU7XG5cbiAgICBob3VnaE1hdGNoZXMucHVzaChtYXRjaGVzW2ldKTtcbiAgfVxuICByZXR1cm4gaG91Z2hNYXRjaGVzO1xufVxuXG5jb25zdCBfbWFwQ29ycmVzcG9uZGVuY2UgPSAoe3F1ZXJ5cG9pbnQsIGtleXBvaW50LCBrZXljZW50ZXJYLCBrZXljZW50ZXJZLCBzY2FsZU9uZU92ZXJMb2dLfSkgPT4ge1xuICAvLyBtYXAgYW5nbGUgdG8gKC1waSwgcGldXG4gIGxldCBhbmdsZSA9IHF1ZXJ5cG9pbnQuYW5nbGUgLSBrZXlwb2ludC5hbmdsZTtcbiAgaWYgKGFuZ2xlIDw9IC1NYXRoLlBJKSBhbmdsZSArPSAyKk1hdGguUEk7XG4gIGVsc2UgaWYgKGFuZ2xlID4gTWF0aC5QSSkgYW5nbGUgLT0gMipNYXRoLlBJO1xuXG4gIGNvbnN0IHNjYWxlID0gcXVlcnlwb2ludC5zY2FsZSAvIGtleXBvaW50LnNjYWxlO1xuXG4gIC8vIDJ4MiBzaW1pbGFyaXR5XG4gIGNvbnN0IGNvcyA9IHNjYWxlICogTWF0aC5jb3MoYW5nbGUpO1xuICBjb25zdCBzaW4gPSBzY2FsZSAqIE1hdGguc2luKGFuZ2xlKTtcbiAgY29uc3QgUyA9IFtjb3MsIC1zaW4sIHNpbiwgY29zXTtcblxuICBjb25zdCB0cCA9IFtcbiAgICBTWzBdICoga2V5cG9pbnQueCArIFNbMV0gKiBrZXlwb2ludC55LFxuICAgIFNbMl0gKiBrZXlwb2ludC54ICsgU1szXSAqIGtleXBvaW50LnlcbiAgXTtcbiAgY29uc3QgdHggPSBxdWVyeXBvaW50LnggLSB0cFswXTtcbiAgY29uc3QgdHkgPSBxdWVyeXBvaW50LnkgLSB0cFsxXTtcblxuICByZXR1cm4ge1xuICAgIHg6IFNbMF0gKiBrZXljZW50ZXJYICsgU1sxXSAqIGtleWNlbnRlclkgKyB0eCxcbiAgICB5OiBTWzJdICoga2V5Y2VudGVyWCArIFNbM10gKiBrZXljZW50ZXJZICsgdHksXG4gICAgYW5nbGU6IGFuZ2xlLFxuICAgIHNjYWxlOiBNYXRoLmxvZyhzY2FsZSkgKiBzY2FsZU9uZU92ZXJMb2dLXG4gIH1cbn1cblxubW9kdWxlLmV4cG9ydHMgPSB7XG4gIGNvbXB1dGVIb3VnaE1hdGNoZXNcbn1cbiIsImNvbnN0IHttYXRjaH0gPSByZXF1aXJlKCcuL21hdGNoaW5nJyk7XG5cbmNsYXNzIE1hdGNoZXIge1xuICBjb25zdHJ1Y3RvcihxdWVyeVdpZHRoLCBxdWVyeUhlaWdodCwgZGVidWdNb2RlID0gZmFsc2UpIHtcbiAgICB0aGlzLnF1ZXJ5V2lkdGggPSBxdWVyeVdpZHRoO1xuICAgIHRoaXMucXVlcnlIZWlnaHQgPSBxdWVyeUhlaWdodDtcbiAgICB0aGlzLmRlYnVnTW9kZSA9IGRlYnVnTW9kZTtcbiAgfVxuXG4gIG1hdGNoRGV0ZWN0aW9uKGtleWZyYW1lcywgZmVhdHVyZVBvaW50cykge1xuICAgIGxldCBkZWJ1Z0V4dHJhID0ge2ZyYW1lczogW119O1xuXG4gICAgbGV0IGJlc3RSZXN1bHQgPSBudWxsO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwga2V5ZnJhbWVzLmxlbmd0aDsgaSsrKSB7XG4gICAgICBjb25zdCB7SCwgbWF0Y2hlcywgZGVidWdFeHRyYTogZnJhbWVEZWJ1Z0V4dHJhfSA9IG1hdGNoKHtrZXlmcmFtZToga2V5ZnJhbWVzW2ldLCBxdWVyeXBvaW50czogZmVhdHVyZVBvaW50cywgcXVlcnl3aWR0aDogdGhpcy5xdWVyeVdpZHRoLCBxdWVyeWhlaWdodDogdGhpcy5xdWVyeUhlaWdodCwgZGVidWdNb2RlOiB0aGlzLmRlYnVnTW9kZX0pO1xuICAgICAgZGVidWdFeHRyYS5mcmFtZXMucHVzaChmcmFtZURlYnVnRXh0cmEpO1xuXG4gICAgICBpZiAoSCkge1xuXHRpZiAoYmVzdFJlc3VsdCA9PT0gbnVsbCB8fCBiZXN0UmVzdWx0Lm1hdGNoZXMubGVuZ3RoIDwgbWF0Y2hlcy5sZW5ndGgpIHtcblx0ICBiZXN0UmVzdWx0ID0ge2tleWZyYW1lSW5kZXg6IGksIEgsIG1hdGNoZXN9O1xuXHR9XG4gICAgICB9XG4gICAgfVxuXG4gICAgaWYgKGJlc3RSZXN1bHQgPT09IG51bGwpIHtcbiAgICAgIHJldHVybiB7a2V5ZnJhbWVJbmRleDogLTEsIGRlYnVnRXh0cmF9O1xuICAgIH1cblxuICAgIGNvbnN0IHNjcmVlbkNvb3JkcyA9IFtdO1xuICAgIGNvbnN0IHdvcmxkQ29vcmRzID0gW107XG4gICAgY29uc3Qga2V5ZnJhbWUgPSBrZXlmcmFtZXNbYmVzdFJlc3VsdC5rZXlmcmFtZUluZGV4XTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGJlc3RSZXN1bHQubWF0Y2hlcy5sZW5ndGg7IGkrKykge1xuICAgICAgY29uc3QgcXVlcnlwb2ludCA9IGJlc3RSZXN1bHQubWF0Y2hlc1tpXS5xdWVyeXBvaW50O1xuICAgICAgY29uc3Qga2V5cG9pbnQgPSBiZXN0UmVzdWx0Lm1hdGNoZXNbaV0ua2V5cG9pbnQ7XG4gICAgICBzY3JlZW5Db29yZHMucHVzaCh7XG4gICAgICAgIHg6IHF1ZXJ5cG9pbnQueCxcbiAgICAgICAgeTogcXVlcnlwb2ludC55LFxuICAgICAgfSlcbiAgICAgIHdvcmxkQ29vcmRzLnB1c2goe1xuICAgICAgICB4OiAoa2V5cG9pbnQueCArIDAuNSkgLyBrZXlmcmFtZS5zY2FsZSxcbiAgICAgICAgeTogKGtleXBvaW50LnkgKyAwLjUpIC8ga2V5ZnJhbWUuc2NhbGUsXG4gICAgICAgIHo6IDAsXG4gICAgICB9KVxuICAgIH1cbiAgICByZXR1cm4ge3NjcmVlbkNvb3Jkcywgd29ybGRDb29yZHMsIGtleWZyYW1lSW5kZXg6IGJlc3RSZXN1bHQua2V5ZnJhbWVJbmRleCwgZGVidWdFeHRyYX07XG4gIH1cbn1cblxubW9kdWxlLmV4cG9ydHMgPSB7XG4gIE1hdGNoZXJcbn1cbiIsImNvbnN0IFRpbnlRdWV1ZSA9IHJlcXVpcmUoJ3RpbnlxdWV1ZScpLmRlZmF1bHQ7XG5jb25zdCB7Y29tcHV0ZTogaGFtbWluZ0NvbXB1dGV9ID0gcmVxdWlyZSgnLi9oYW1taW5nLWRpc3RhbmNlLmpzJyk7XG5jb25zdCB7Y29tcHV0ZUhvdWdoTWF0Y2hlc30gPSByZXF1aXJlKCcuL2hvdWdoLmpzJyk7XG5jb25zdCB7Y29tcHV0ZUhvbW9ncmFwaHl9ID0gcmVxdWlyZSgnLi9yYW5zYWNIb21vZ3JhcGh5LmpzJyk7XG5jb25zdCB7bXVsdGlwbHlQb2ludEhvbW9ncmFwaHlJbmhvbW9nZW5vdXMsIG1hdHJpeEludmVyc2UzM30gPSByZXF1aXJlKCcuLi91dGlscy9nZW9tZXRyeS5qcycpO1xuXG5jb25zdCBJTkxJRVJfVEhSRVNIT0xEID0gMztcbi8vY29uc3QgTUlOX05VTV9JTkxJRVJTID0gODsgIC8vZGVmYXVsdFxuY29uc3QgTUlOX05VTV9JTkxJRVJTID0gNjtcbmNvbnN0IENMVVNURVJfTUFYX1BPUCA9IDg7XG5jb25zdCBIQU1NSU5HX1RIUkVTSE9MRCA9IDAuNztcblxuLy8gbWF0Y2ggbGlzdCBvZiBxdWVycG9pbnRzIGFnYWluc3QgcHJlLWJ1aWx0IGxpc3Qgb2Yga2V5ZnJhbWVzXG5jb25zdCBtYXRjaCA9ICh7a2V5ZnJhbWUsIHF1ZXJ5cG9pbnRzLCBxdWVyeXdpZHRoLCBxdWVyeWhlaWdodCwgZGVidWdNb2RlfSkgPT4ge1xuICBsZXQgZGVidWdFeHRyYSA9IHt9O1xuXG4gIGNvbnN0IG1hdGNoZXMgPSBbXTtcbiAgZm9yIChsZXQgaiA9IDA7IGogPCBxdWVyeXBvaW50cy5sZW5ndGg7IGorKykge1xuICAgIGNvbnN0IHF1ZXJ5cG9pbnQgPSBxdWVyeXBvaW50c1tqXTtcbiAgICBjb25zdCBrZXlwb2ludHMgPSBxdWVyeXBvaW50Lm1heGltYT8ga2V5ZnJhbWUubWF4aW1hUG9pbnRzOiBrZXlmcmFtZS5taW5pbWFQb2ludHM7XG4gICAgaWYgKGtleXBvaW50cy5sZW5ndGggPT09IDApIGNvbnRpbnVlO1xuXG4gICAgY29uc3Qgcm9vdE5vZGUgPSBxdWVyeXBvaW50Lm1heGltYT8ga2V5ZnJhbWUubWF4aW1hUG9pbnRzQ2x1c3Rlci5yb290Tm9kZToga2V5ZnJhbWUubWluaW1hUG9pbnRzQ2x1c3Rlci5yb290Tm9kZTtcblxuICAgIGNvbnN0IGtleXBvaW50SW5kZXhlcyA9IFtdO1xuICAgIGNvbnN0IHF1ZXVlID0gbmV3IFRpbnlRdWV1ZShbXSwgKGExLCBhMikgPT4ge3JldHVybiBhMS5kIC0gYTIuZH0pO1xuXG4gICAgLy8gcXVlcnkgYWxsIHBvdGVudGlhbCBrZXlwb2ludHNcbiAgICBfcXVlcnkoe25vZGU6IHJvb3ROb2RlLCBrZXlwb2ludHMsIHF1ZXJ5cG9pbnQsIHF1ZXVlLCBrZXlwb2ludEluZGV4ZXMsIG51bVBvcDogMH0pO1xuXG4gICAgbGV0IGJlc3RJbmRleCA9IC0xO1xuICAgIGxldCBiZXN0RDEgPSBOdW1iZXIuTUFYX1NBRkVfSU5URUdFUjtcbiAgICBsZXQgYmVzdEQyID0gTnVtYmVyLk1BWF9TQUZFX0lOVEVHRVI7XG5cbiAgICBmb3IgKGxldCBrID0gMDsgayA8IGtleXBvaW50SW5kZXhlcy5sZW5ndGg7IGsrKykge1xuICAgICAgY29uc3Qga2V5cG9pbnQgPSBrZXlwb2ludHNba2V5cG9pbnRJbmRleGVzW2tdXTtcblxuICAgICAgY29uc3QgZCA9IGhhbW1pbmdDb21wdXRlKHt2MToga2V5cG9pbnQuZGVzY3JpcHRvcnMsIHYyOiBxdWVyeXBvaW50LmRlc2NyaXB0b3JzfSk7XG4gICAgICBpZiAoZCA8IGJlc3REMSkge1xuXHRiZXN0RDIgPSBiZXN0RDE7XG5cdGJlc3REMSA9IGQ7XG5cdGJlc3RJbmRleCA9IGtleXBvaW50SW5kZXhlc1trXTtcbiAgICAgIH0gZWxzZSBpZiAoZCA8IGJlc3REMikge1xuXHRiZXN0RDIgPSBkO1xuICAgICAgfVxuICAgIH1cbiAgICBpZiAoYmVzdEluZGV4ICE9PSAtMSAmJiAoYmVzdEQyID09PSBOdW1iZXIuTUFYX1NBRkVfSU5URUdFUiB8fCAoMS4wICogYmVzdEQxIC8gYmVzdEQyKSA8IEhBTU1JTkdfVEhSRVNIT0xEKSkge1xuICAgICAgbWF0Y2hlcy5wdXNoKHtxdWVyeXBvaW50LCBrZXlwb2ludDoga2V5cG9pbnRzW2Jlc3RJbmRleF19KTtcbiAgICB9XG4gIH1cblxuICBpZiAoZGVidWdNb2RlKSB7XG4gICAgZGVidWdFeHRyYS5tYXRjaGVzID0gbWF0Y2hlcztcbiAgfVxuXG4gIGlmIChtYXRjaGVzLmxlbmd0aCA8IE1JTl9OVU1fSU5MSUVSUykgcmV0dXJuIHtkZWJ1Z0V4dHJhfTtcblxuICBjb25zdCBob3VnaE1hdGNoZXMgPSBjb21wdXRlSG91Z2hNYXRjaGVzKHtcbiAgICBrZXl3aWR0aDoga2V5ZnJhbWUud2lkdGgsXG4gICAga2V5aGVpZ2h0OiBrZXlmcmFtZS5oZWlnaHQsXG4gICAgcXVlcnl3aWR0aCxcbiAgICBxdWVyeWhlaWdodCxcbiAgICBtYXRjaGVzLFxuICB9KTtcblxuICBpZiAoZGVidWdNb2RlKSB7XG4gICAgZGVidWdFeHRyYS5ob3VnaE1hdGNoZXMgPSBob3VnaE1hdGNoZXM7XG4gIH1cblxuICBjb25zdCBIID0gY29tcHV0ZUhvbW9ncmFwaHkoe1xuICAgIHNyY1BvaW50czogaG91Z2hNYXRjaGVzLm1hcCgobSkgPT4gW20ua2V5cG9pbnQueCwgbS5rZXlwb2ludC55XSksXG4gICAgZHN0UG9pbnRzOiBob3VnaE1hdGNoZXMubWFwKChtKSA9PiBbbS5xdWVyeXBvaW50LngsIG0ucXVlcnlwb2ludC55XSksXG4gICAga2V5ZnJhbWUsXG4gIH0pO1xuXG4gIGlmIChIID09PSBudWxsKSByZXR1cm4ge2RlYnVnRXh0cmF9O1xuXG4gIGNvbnN0IGlubGllck1hdGNoZXMgPSBfZmluZElubGllck1hdGNoZXMoe1xuICAgIEgsXG4gICAgbWF0Y2hlczogaG91Z2hNYXRjaGVzLFxuICAgIHRocmVzaG9sZDogSU5MSUVSX1RIUkVTSE9MRFxuICB9KTtcbiAgXG4gIGlmIChkZWJ1Z01vZGUpIHtcbiAgICBkZWJ1Z0V4dHJhLmlubGllck1hdGNoZXMgPSBpbmxpZXJNYXRjaGVzO1xuICB9XG5cbiAgaWYgKGlubGllck1hdGNoZXMubGVuZ3RoIDwgTUlOX05VTV9JTkxJRVJTKSByZXR1cm4ge2RlYnVnRXh0cmF9OyBcblxuICAvLyBkbyBhbm90aGVyIGxvb3Agb2YgbWF0Y2ggdXNpbmcgdGhlIGhvbW9ncmFwaHlcbiAgY29uc3QgSEludiA9IG1hdHJpeEludmVyc2UzMyhILCAwLjAwMDAxKTtcbiAgY29uc3QgZFRocmVzaG9sZDIgPSAxMCAqIDEwO1xuICBjb25zdCBtYXRjaGVzMiA9IFtdO1xuICBmb3IgKGxldCBqID0gMDsgaiA8IHF1ZXJ5cG9pbnRzLmxlbmd0aDsgaisrKSB7XG4gICAgY29uc3QgcXVlcnlwb2ludCA9IHF1ZXJ5cG9pbnRzW2pdO1xuICAgIGNvbnN0IG1hcHF1ZXJ5cG9pbnQgPSBtdWx0aXBseVBvaW50SG9tb2dyYXBoeUluaG9tb2dlbm91cyhbcXVlcnlwb2ludC54LCBxdWVyeXBvaW50LnldLCBISW52KTtcblxuICAgIGxldCBiZXN0SW5kZXggPSAtMTtcbiAgICBsZXQgYmVzdEQxID0gTnVtYmVyLk1BWF9TQUZFX0lOVEVHRVI7XG4gICAgbGV0IGJlc3REMiA9IE51bWJlci5NQVhfU0FGRV9JTlRFR0VSO1xuXG4gICAgY29uc3Qga2V5cG9pbnRzID0gcXVlcnlwb2ludC5tYXhpbWE/IGtleWZyYW1lLm1heGltYVBvaW50czoga2V5ZnJhbWUubWluaW1hUG9pbnRzO1xuXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCBrZXlwb2ludHMubGVuZ3RoOyBrKyspIHtcbiAgICAgIGNvbnN0IGtleXBvaW50ID0ga2V5cG9pbnRzW2tdO1xuXG4gICAgICAvLyBjaGVjayBkaXN0YW5jZSB0aHJlc2hvbGRcbiAgICAgIGNvbnN0IGQyID0gKGtleXBvaW50LnggLSBtYXBxdWVyeXBvaW50WzBdKSAqIChrZXlwb2ludC54IC0gbWFwcXVlcnlwb2ludFswXSlcblx0XHQrIChrZXlwb2ludC55IC0gbWFwcXVlcnlwb2ludFsxXSkgKiAoa2V5cG9pbnQueSAtIG1hcHF1ZXJ5cG9pbnRbMV0pO1xuICAgICAgaWYgKGQyID4gZFRocmVzaG9sZDIpIGNvbnRpbnVlO1xuXG4gICAgICBjb25zdCBkID0gaGFtbWluZ0NvbXB1dGUoe3YxOiBrZXlwb2ludC5kZXNjcmlwdG9ycywgdjI6IHF1ZXJ5cG9pbnQuZGVzY3JpcHRvcnN9KTtcbiAgICAgIGlmIChkIDwgYmVzdEQxKSB7XG5cdGJlc3REMiA9IGJlc3REMTtcblx0YmVzdEQxID0gZDtcblx0YmVzdEluZGV4ID0gaztcbiAgICAgIH0gZWxzZSBpZiAoZCA8IGJlc3REMikge1xuXHRiZXN0RDIgPSBkO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChiZXN0SW5kZXggIT09IC0xICYmIChiZXN0RDIgPT09IE51bWJlci5NQVhfU0FGRV9JTlRFR0VSIHx8ICgxLjAgKiBiZXN0RDEgLyBiZXN0RDIpIDwgSEFNTUlOR19USFJFU0hPTEQpKSB7XG4gICAgICBtYXRjaGVzMi5wdXNoKHtxdWVyeXBvaW50LCBrZXlwb2ludDoga2V5cG9pbnRzW2Jlc3RJbmRleF19KTtcbiAgICB9XG4gIH1cblxuICBpZiAoZGVidWdNb2RlKSB7XG4gICAgZGVidWdFeHRyYS5tYXRjaGVzMiA9IG1hdGNoZXMyO1xuICB9XG5cbiAgY29uc3QgaG91Z2hNYXRjaGVzMiA9IGNvbXB1dGVIb3VnaE1hdGNoZXMoe1xuICAgIGtleXdpZHRoOiBrZXlmcmFtZS53aWR0aCxcbiAgICBrZXloZWlnaHQ6IGtleWZyYW1lLmhlaWdodCxcbiAgICBxdWVyeXdpZHRoLFxuICAgIHF1ZXJ5aGVpZ2h0LFxuICAgIG1hdGNoZXM6IG1hdGNoZXMyLFxuICB9KTtcblxuICBpZiAoZGVidWdNb2RlKSB7XG4gICAgZGVidWdFeHRyYS5ob3VnaE1hdGNoZXMyID0gaG91Z2hNYXRjaGVzMjtcbiAgfVxuXG4gIGNvbnN0IEgyID0gY29tcHV0ZUhvbW9ncmFwaHkoe1xuICAgIHNyY1BvaW50czogaG91Z2hNYXRjaGVzMi5tYXAoKG0pID0+IFttLmtleXBvaW50LngsIG0ua2V5cG9pbnQueV0pLFxuICAgIGRzdFBvaW50czogaG91Z2hNYXRjaGVzMi5tYXAoKG0pID0+IFttLnF1ZXJ5cG9pbnQueCwgbS5xdWVyeXBvaW50LnldKSxcbiAgICBrZXlmcmFtZSxcbiAgfSk7XG5cbiAgaWYgKEgyID09PSBudWxsKSByZXR1cm4ge2RlYnVnRXh0cmF9O1xuXG4gIGNvbnN0IGlubGllck1hdGNoZXMyID0gX2ZpbmRJbmxpZXJNYXRjaGVzKHtcbiAgICBIOiBIMixcbiAgICBtYXRjaGVzOiBob3VnaE1hdGNoZXMyLFxuICAgIHRocmVzaG9sZDogSU5MSUVSX1RIUkVTSE9MRFxuICB9KTtcblxuICBpZiAoZGVidWdNb2RlKSB7XG4gICAgZGVidWdFeHRyYS5pbmxpZXJNYXRjaGVzMiA9IGlubGllck1hdGNoZXMyO1xuICB9XG5cbiAgcmV0dXJuIHtIOiBIMiwgbWF0Y2hlczogaW5saWVyTWF0Y2hlczIsIGRlYnVnRXh0cmF9O1xufTtcblxuY29uc3QgX3F1ZXJ5ID0gKHtub2RlLCBrZXlwb2ludHMsIHF1ZXJ5cG9pbnQsIHF1ZXVlLCBrZXlwb2ludEluZGV4ZXMsIG51bVBvcH0pID0+IHtcbiAgaWYgKG5vZGUubGVhZikge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbm9kZS5wb2ludEluZGV4ZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgIGtleXBvaW50SW5kZXhlcy5wdXNoKG5vZGUucG9pbnRJbmRleGVzW2ldKTtcbiAgICB9XG4gICAgcmV0dXJuO1xuICB9XG5cbiAgY29uc3QgZGlzdGFuY2VzID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbm9kZS5jaGlsZHJlbi5sZW5ndGg7IGkrKykge1xuICAgIGNvbnN0IGNoaWxkTm9kZSA9IG5vZGUuY2hpbGRyZW5baV07XG4gICAgY29uc3QgY2VudGVyUG9pbnRJbmRleCA9IGNoaWxkTm9kZS5jZW50ZXJQb2ludEluZGV4O1xuICAgIGNvbnN0IGQgPSBoYW1taW5nQ29tcHV0ZSh7djE6IGtleXBvaW50c1tjZW50ZXJQb2ludEluZGV4XS5kZXNjcmlwdG9ycywgdjI6IHF1ZXJ5cG9pbnQuZGVzY3JpcHRvcnN9KTtcbiAgICBkaXN0YW5jZXMucHVzaChkKTtcbiAgfVxuXG4gIGxldCBtaW5EID0gTnVtYmVyLk1BWF9TQUZFX0lOVEVHRVI7XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbm9kZS5jaGlsZHJlbi5sZW5ndGg7IGkrKykge1xuICAgIG1pbkQgPSBNYXRoLm1pbihtaW5ELCBkaXN0YW5jZXNbaV0pO1xuICB9XG5cbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBub2RlLmNoaWxkcmVuLmxlbmd0aDsgaSsrKSB7XG4gICAgaWYgKGRpc3RhbmNlc1tpXSAhPT0gbWluRCkge1xuICAgICAgcXVldWUucHVzaCh7bm9kZTogbm9kZS5jaGlsZHJlbltpXSwgZDogZGlzdGFuY2VzW2ldfSk7XG4gICAgfVxuICB9XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbm9kZS5jaGlsZHJlbi5sZW5ndGg7IGkrKykge1xuICAgIGlmIChkaXN0YW5jZXNbaV0gPT09IG1pbkQpIHtcbiAgICAgIF9xdWVyeSh7bm9kZTogbm9kZS5jaGlsZHJlbltpXSwga2V5cG9pbnRzLCBxdWVyeXBvaW50LCBxdWV1ZSwga2V5cG9pbnRJbmRleGVzLCBudW1Qb3B9KTtcbiAgICB9XG4gIH1cblxuICBpZiAobnVtUG9wIDwgQ0xVU1RFUl9NQVhfUE9QICYmIHF1ZXVlLmxlbmd0aCA+IDApIHtcbiAgICBjb25zdCB7bm9kZSwgZH0gPSBxdWV1ZS5wb3AoKTtcbiAgICBudW1Qb3AgKz0gMTtcbiAgICBfcXVlcnkoe25vZGUsIGtleXBvaW50cywgcXVlcnlwb2ludCwgcXVldWUsIGtleXBvaW50SW5kZXhlcywgbnVtUG9wfSk7XG4gIH1cbn07XG5cbmNvbnN0IF9maW5kSW5saWVyTWF0Y2hlcyA9IChvcHRpb25zKSA9PiB7XG4gIGNvbnN0IHtILCBtYXRjaGVzLCB0aHJlc2hvbGR9ID0gb3B0aW9ucztcblxuICBjb25zdCB0aHJlc2hvbGQyID0gdGhyZXNob2xkICogdGhyZXNob2xkO1xuXG4gIGNvbnN0IGdvb2RNYXRjaGVzID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbWF0Y2hlcy5sZW5ndGg7IGkrKykge1xuICAgIGNvbnN0IHF1ZXJ5cG9pbnQgPSBtYXRjaGVzW2ldLnF1ZXJ5cG9pbnQ7XG4gICAgY29uc3Qga2V5cG9pbnQgPSBtYXRjaGVzW2ldLmtleXBvaW50O1xuICAgIGNvbnN0IG1wID0gbXVsdGlwbHlQb2ludEhvbW9ncmFwaHlJbmhvbW9nZW5vdXMoW2tleXBvaW50LngsIGtleXBvaW50LnldLCBIKTtcbiAgICBjb25zdCBkMiA9IChtcFswXSAtIHF1ZXJ5cG9pbnQueCkgKiAobXBbMF0gLSBxdWVyeXBvaW50LngpICsgKG1wWzFdIC0gcXVlcnlwb2ludC55KSAqIChtcFsxXSAtIHF1ZXJ5cG9pbnQueSk7XG4gICAgaWYgKGQyIDw9IHRocmVzaG9sZDIpIHtcbiAgICAgIGdvb2RNYXRjaGVzLnB1c2goIG1hdGNoZXNbaV0gKTtcbiAgICB9XG4gIH1cbiAgcmV0dXJuIGdvb2RNYXRjaGVzO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IHtcbiAgbWF0Y2hcbn1cbiIsImNvbnN0IHtNYXRyaXgsIGludmVyc2V9ID0gcmVxdWlyZSgnbWwtbWF0cml4Jyk7XG5jb25zdCB7Y3JlYXRlUmFuZG9taXplcn0gPSByZXF1aXJlKCcuLi91dGlscy9yYW5kb21pemVyLmpzJyk7XG5jb25zdCB7cXVhZHJpbGF0ZXJhbENvbnZleCwgbWF0cml4SW52ZXJzZTMzLCBzbWFsbGVzdFRyaWFuZ2xlQXJlYSwgbXVsdGlwbHlQb2ludEhvbW9ncmFwaHlJbmhvbW9nZW5vdXMsIGNoZWNrVGhyZWVQb2ludHNDb25zaXN0ZW50LCBjaGVja0ZvdXJQb2ludHNDb25zaXN0ZW50LCBkZXRlcm1pbmFudH0gPSByZXF1aXJlKCcuLi91dGlscy9nZW9tZXRyeS5qcycpO1xuY29uc3Qge3NvbHZlSG9tb2dyYXBoeX0gPSByZXF1aXJlKCcuLi91dGlscy9ob21vZ3JhcGh5Jyk7XG5cbmNvbnN0IENBVUNIWV9TQ0FMRSA9IDAuMDE7XG5jb25zdCBDSFVOS19TSVpFID0gMTA7XG5jb25zdCBOVU1fSFlQT1RIRVNFUyA9IDIwO1xuY29uc3QgTlVNX0hZUE9USEVTRVNfUVVJQ0sgPSAxMDtcblxuLy8gVXNpbmcgUkFOU0FDIHRvIGVzdGltYXRlIGhvbW9ncmFwaHlcbmNvbnN0IGNvbXB1dGVIb21vZ3JhcGh5ID0gKG9wdGlvbnMpID0+IHtcbiAgY29uc3Qge3NyY1BvaW50cywgZHN0UG9pbnRzLCBrZXlmcmFtZSwgcXVpY2tNb2RlfSA9IG9wdGlvbnM7XG5cbiAgLy8gdGVzdFBvaW50cyBpcyBmb3VyIGNvcm5lcnMgb2Yga2V5ZnJhbWVcbiAgY29uc3QgdGVzdFBvaW50cyA9IFtcbiAgICBbMCwgMF0sXG4gICAgW2tleWZyYW1lLndpZHRoLCAwXSxcbiAgICBba2V5ZnJhbWUud2lkdGgsIGtleWZyYW1lLmhlaWdodF0sXG4gICAgWzAsIGtleWZyYW1lLmhlaWdodF1cbiAgXVxuXG4gIGNvbnN0IHNhbXBsZVNpemUgPSA0OyAvLyB1c2UgZm91ciBwb2ludHMgdG8gY29tcHV0ZSBob21vZ3JhcGh5XG4gIGlmIChzcmNQb2ludHMubGVuZ3RoIDwgc2FtcGxlU2l6ZSkgcmV0dXJuIG51bGw7XG5cbiAgY29uc3Qgc2NhbGUgPSBDQVVDSFlfU0NBTEU7XG4gIGNvbnN0IG9uZU92ZXJTY2FsZTIgPSAxLjAgLyAoc2NhbGUgKiBzY2FsZSk7XG4gIGNvbnN0IGNodWNrU2l6ZSA9IE1hdGgubWluKENIVU5LX1NJWkUsIHNyY1BvaW50cy5sZW5ndGgpO1xuXG4gIGNvbnN0IHJhbmRvbWl6ZXIgPSBjcmVhdGVSYW5kb21pemVyKCk7XG5cbiAgY29uc3QgcGVybSA9IFtdO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IHNyY1BvaW50cy5sZW5ndGg7IGkrKykge1xuICAgIHBlcm1baV0gPSBpO1xuICB9XG5cbiAgcmFuZG9taXplci5hcnJheVNodWZmbGUoe2FycjogcGVybSwgc2FtcGxlU2l6ZTogcGVybS5sZW5ndGh9KTtcblxuICBjb25zdCBudW1IeXBvdGhlc2lzID0gcXVpY2tNb2RlPyBOVU1fSFlQT1RIRVNFU19RVUlDSzogTlVNX0hZUE9USEVTRVM7XG4gIGNvbnN0IG1heFRyaWFscyA9IG51bUh5cG90aGVzaXMgKiAyO1xuXG4gIC8vIGJ1aWxkIG51bWVyb3VzIGh5cG90aGVzZXMgYnkgcmFuZG9taW5nIGRyYXcgZm91ciBwb2ludHNcbiAgLy8gVE9ETzogb3B0aW1pemU6IGlmIG51bWJlciBvZiBwb2ludHMgaXMgbGVzcyB0aGFuIGNlcnRhaW4gbnVtYmVyLCBjYW4gYnJ1dGUgZm9yY2UgYWxsIGNvbWJpbmF0aW9uc1xuICBsZXQgdHJpYWwgPSAwO1xuICBjb25zdCBIcyA9IFtdO1xuICB3aGlsZSAodHJpYWwgPCBtYXhUcmlhbHMgJiYgSHMubGVuZ3RoIDwgbnVtSHlwb3RoZXNpcykge1xuICAgIHRyaWFsICs9MTtcblxuICAgIHJhbmRvbWl6ZXIuYXJyYXlTaHVmZmxlKHthcnI6IHBlcm0sIHNhbXBsZVNpemU6IHNhbXBsZVNpemV9KTtcblxuICAgIC8vIHRoZWlyIHJlbGF0aXZlIHBvc2l0aW9ucyBtYXRjaCBlYWNoIG90aGVyXG4gICAgaWYgKCFjaGVja0ZvdXJQb2ludHNDb25zaXN0ZW50KFxuICAgICAgc3JjUG9pbnRzW3Blcm1bMF1dLCBzcmNQb2ludHNbcGVybVsxXV0sIHNyY1BvaW50c1twZXJtWzJdXSwgc3JjUG9pbnRzW3Blcm1bM11dLFxuICAgICAgZHN0UG9pbnRzW3Blcm1bMF1dLCBkc3RQb2ludHNbcGVybVsxXV0sIGRzdFBvaW50c1twZXJtWzJdXSwgZHN0UG9pbnRzW3Blcm1bM11dKSkge1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgY29uc3QgSCA9IHNvbHZlSG9tb2dyYXBoeShcbiAgICAgIFtzcmNQb2ludHNbcGVybVswXV0sIHNyY1BvaW50c1twZXJtWzFdXSwgc3JjUG9pbnRzW3Blcm1bMl1dLCBzcmNQb2ludHNbcGVybVszXV1dLFxuICAgICAgW2RzdFBvaW50c1twZXJtWzBdXSwgZHN0UG9pbnRzW3Blcm1bMV1dLCBkc3RQb2ludHNbcGVybVsyXV0sIGRzdFBvaW50c1twZXJtWzNdXV0sXG4gICAgKTtcbiAgICBpZiAoSCA9PT0gbnVsbCkgY29udGludWU7XG5cbiAgICBpZighX2NoZWNrSG9tb2dyYXBoeVBvaW50c0dlb21ldHJpY2FsbHlDb25zaXN0ZW50KHtILCB0ZXN0UG9pbnRzfSkpIHtcbiAgICAgIGNvbnRpbnVlO1xuICAgIH1cblxuICAgIEhzLnB1c2goSCk7XG4gIH1cblxuICBpZiAoSHMubGVuZ3RoID09PSAwKSByZXR1cm4gbnVsbDtcblxuICAvLyBwaWNrIHRoZSBiZXN0IGh5cG90aGVzaXNcbiAgY29uc3QgaHlwb3RoZXNlcyA9IFtdO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IEhzLmxlbmd0aDsgaSsrKSB7XG4gICAgaHlwb3RoZXNlcy5wdXNoKHtcbiAgICAgIEg6IEhzW2ldLFxuICAgICAgY29zdDogMFxuICAgIH0pXG4gIH1cblxuICBsZXQgY3VyQ2h1Y2tTaXplID0gY2h1Y2tTaXplO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IHNyY1BvaW50cy5sZW5ndGggJiYgaHlwb3RoZXNlcy5sZW5ndGggPiAyOyBpICs9IGN1ckNodWNrU2l6ZSkge1xuICAgIGN1ckNodWNrU2l6ZSA9IE1hdGgubWluKGNodWNrU2l6ZSwgc3JjUG9pbnRzLmxlbmd0aCAtIGkpO1xuICAgIGxldCBjaHVja0VuZCA9IGkgKyBjdXJDaHVja1NpemU7XG5cbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IGh5cG90aGVzZXMubGVuZ3RoOyBqKyspIHtcbiAgICAgIGZvciAobGV0IGsgPSBpOyBrIDwgY2h1Y2tFbmQ7IGsrKykge1xuICAgICAgICBjb25zdCBjb3N0ID0gX2NhdWNoeVByb2plY3RpdmVSZXByb2plY3Rpb25Db3N0KHtIOiBoeXBvdGhlc2VzW2pdLkgsIHNyY1BvaW50OiBzcmNQb2ludHNba10sIGRzdFBvaW50OiBkc3RQb2ludHNba10sIG9uZU92ZXJTY2FsZTJ9KTtcbiAgICAgICAgaHlwb3RoZXNlc1tqXS5jb3N0ICs9IGNvc3Q7XG4gICAgICB9XG4gICAgfVxuXG4gICAgaHlwb3RoZXNlcy5zb3J0KChoMSwgaDIpID0+IHtyZXR1cm4gaDEuY29zdCAtIGgyLmNvc3R9KTtcbiAgICBoeXBvdGhlc2VzLnNwbGljZSgtTWF0aC5mbG9vcigoaHlwb3RoZXNlcy5sZW5ndGgrMSkvMikpOyAvLyBrZWVwIHRoZSBiZXN0IGhhbGZcbiAgfVxuXG4gIGxldCBmaW5hbEggPSBudWxsO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGh5cG90aGVzZXMubGVuZ3RoOyBpKyspIHtcbiAgICBjb25zdCBIID0gX25vcm1hbGl6ZUhvbW9ncmFwaHkoe2luSDogaHlwb3RoZXNlc1tpXS5IfSk7XG4gICAgaWYgKF9jaGVja0hldXJpc3RpY3Moe0g6IEgsIHRlc3RQb2ludHMsIGtleWZyYW1lfSkpIHtcbiAgICAgIGZpbmFsSCA9IEg7XG4gICAgICBicmVhaztcbiAgICB9XG4gIH1cbiAgcmV0dXJuIGZpbmFsSDtcbn1cblxuY29uc3QgX2NoZWNrSGV1cmlzdGljcyA9ICh7SCwgdGVzdFBvaW50cywga2V5ZnJhbWV9KSA9PiB7XG4gIGNvbnN0IEhJbnYgPSBtYXRyaXhJbnZlcnNlMzMoSCwgMC4wMDAwMSk7XG4gIGlmIChISW52ID09PSBudWxsKSByZXR1cm4gZmFsc2U7XG5cbiAgY29uc3QgbXAgPSBbXVxuICBmb3IgKGxldCBpID0gMDsgaSA8IHRlc3RQb2ludHMubGVuZ3RoOyBpKyspIHsgLy8gNCB0ZXN0IHBvaW50cywgY29ybmVyIG9mIGtleWZyYW1lXG4gICAgbXAucHVzaChtdWx0aXBseVBvaW50SG9tb2dyYXBoeUluaG9tb2dlbm91cyh0ZXN0UG9pbnRzW2ldLCBISW52KSk7XG4gIH1cbiAgY29uc3Qgc21hbGxBcmVhID0gc21hbGxlc3RUcmlhbmdsZUFyZWEobXBbMF0sIG1wWzFdLCBtcFsyXSwgbXBbM10pO1xuXG4gIGlmIChzbWFsbEFyZWEgPCBrZXlmcmFtZS53aWR0aCAqIGtleWZyYW1lLmhlaWdodCAqIDAuMDAwMSkgcmV0dXJuIGZhbHNlO1xuXG4gIGlmICghcXVhZHJpbGF0ZXJhbENvbnZleChtcFswXSwgbXBbMV0sIG1wWzJdLCBtcFszXSkpIHJldHVybiBmYWxzZTtcblxuICByZXR1cm4gdHJ1ZTtcbn1cblxuY29uc3QgX25vcm1hbGl6ZUhvbW9ncmFwaHkgPSAoe2luSH0pID0+IHtcbiAgY29uc3Qgb25lT3ZlciA9IDEuMCAvIGluSFs4XTtcblxuICBjb25zdCBIID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgODsgaSsrKSB7XG4gICAgSFtpXSA9IGluSFtpXSAqIG9uZU92ZXI7XG4gIH1cbiAgSFs4XSA9IDEuMDtcbiAgcmV0dXJuIEg7XG59XG5cbmNvbnN0IF9jYXVjaHlQcm9qZWN0aXZlUmVwcm9qZWN0aW9uQ29zdCA9ICh7SCwgc3JjUG9pbnQsIGRzdFBvaW50LCBvbmVPdmVyU2NhbGUyfSkgPT4ge1xuICBjb25zdCB4ID0gbXVsdGlwbHlQb2ludEhvbW9ncmFwaHlJbmhvbW9nZW5vdXMoc3JjUG9pbnQsIEgpO1xuICBjb25zdCBmID1bXG4gICAgeFswXSAtIGRzdFBvaW50WzBdLFxuICAgIHhbMV0gLSBkc3RQb2ludFsxXVxuICBdO1xuICByZXR1cm4gTWF0aC5sb2coMSArIChmWzBdKmZbMF0rZlsxXSpmWzFdKSAqIG9uZU92ZXJTY2FsZTIpO1xufVxuXG5jb25zdCBfY2hlY2tIb21vZ3JhcGh5UG9pbnRzR2VvbWV0cmljYWxseUNvbnNpc3RlbnQgPSAoe0gsIHRlc3RQb2ludHN9KSA9PiB7XG4gIGNvbnN0IG1hcHBlZFBvaW50cyA9IFtdO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IHRlc3RQb2ludHMubGVuZ3RoOyBpKyspIHtcbiAgICBtYXBwZWRQb2ludHNbaV0gPSBtdWx0aXBseVBvaW50SG9tb2dyYXBoeUluaG9tb2dlbm91cyh0ZXN0UG9pbnRzW2ldLCBIKTtcbiAgfVxuICBmb3IgKGxldCBpID0gMDsgaSA8IHRlc3RQb2ludHMubGVuZ3RoOyBpKyspIHtcbiAgICBjb25zdCBpMSA9IGk7XG4gICAgY29uc3QgaTIgPSAoaSsxKSAlIHRlc3RQb2ludHMubGVuZ3RoO1xuICAgIGNvbnN0IGkzID0gKGkrMikgJSB0ZXN0UG9pbnRzLmxlbmd0aDtcbiAgICBpZiAoIWNoZWNrVGhyZWVQb2ludHNDb25zaXN0ZW50KFxuICAgICAgdGVzdFBvaW50c1tpMV0sIHRlc3RQb2ludHNbaTJdLCB0ZXN0UG9pbnRzW2kzXSxcbiAgICAgIG1hcHBlZFBvaW50c1tpMV0sIG1hcHBlZFBvaW50c1tpMl0sIG1hcHBlZFBvaW50c1tpM10pKSByZXR1cm4gZmFsc2U7XG4gIH1cbiAgcmV0dXJuIHRydWU7XG59XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuICBjb21wdXRlSG9tb2dyYXBoeSxcbn1cbiIsIi8vIGNoZWNrIHdoaWNoIHNpZGUgcG9pbnQgQyBvbiB0aGUgbGluZSBmcm9tIEEgdG8gQlxuY29uc3QgbGluZVBvaW50U2lkZSA9IChBLCBCLCBDKSA9PiB7XG4gIHJldHVybiAoKEJbMF0tQVswXSkqKENbMV0tQVsxXSktKEJbMV0tQVsxXSkqKENbMF0tQVswXSkpO1xufVxuXG4vLyBzcmNQb2ludHMsIGRzdFBvaW50czogYXJyYXkgb2YgZm91ciBlbGVtZW50cyBbeCwgeV1cbmNvbnN0IGNoZWNrRm91clBvaW50c0NvbnNpc3RlbnQgPSAoeDEsIHgyLCB4MywgeDQsIHgxcCwgeDJwLCB4M3AsIHg0cCkgPT4ge1xuICBpZiAoKGxpbmVQb2ludFNpZGUoeDEsIHgyLCB4MykgPiAwKSAhPT0gKGxpbmVQb2ludFNpZGUoeDFwLCB4MnAsIHgzcCkgPiAwKSkgcmV0dXJuIGZhbHNlO1xuICBpZiAoKGxpbmVQb2ludFNpZGUoeDIsIHgzLCB4NCkgPiAwKSAhPT0gKGxpbmVQb2ludFNpZGUoeDJwLCB4M3AsIHg0cCkgPiAwKSkgcmV0dXJuIGZhbHNlO1xuICBpZiAoKGxpbmVQb2ludFNpZGUoeDMsIHg0LCB4MSkgPiAwKSAhPT0gKGxpbmVQb2ludFNpZGUoeDNwLCB4NHAsIHgxcCkgPiAwKSkgcmV0dXJuIGZhbHNlO1xuICBpZiAoKGxpbmVQb2ludFNpZGUoeDQsIHgxLCB4MikgPiAwKSAhPT0gKGxpbmVQb2ludFNpZGUoeDRwLCB4MXAsIHgycCkgPiAwKSkgcmV0dXJuIGZhbHNlO1xuICByZXR1cm4gdHJ1ZTtcbn1cblxuY29uc3QgY2hlY2tUaHJlZVBvaW50c0NvbnNpc3RlbnQgPSAoeDEsIHgyLCB4MywgeDFwLCB4MnAsIHgzcCkgPT4ge1xuICBpZiAoKGxpbmVQb2ludFNpZGUoeDEsIHgyLCB4MykgPiAwKSAhPT0gKGxpbmVQb2ludFNpZGUoeDFwLCB4MnAsIHgzcCkgPiAwKSkgcmV0dXJuIGZhbHNlO1xuICByZXR1cm4gdHJ1ZTtcbn1cblxuY29uc3QgZGV0ZXJtaW5hbnQgPSAoQSkgPT4ge1xuICBjb25zdCBDMSA9ICBBWzRdICogQVs4XSAtIEFbNV0gKiBBWzddO1xuICBjb25zdCBDMiA9ICBBWzNdICogQVs4XSAtIEFbNV0gKiBBWzZdO1xuICBjb25zdCBDMyA9ICBBWzNdICogQVs3XSAtIEFbNF0gKiBBWzZdO1xuICByZXR1cm4gQVswXSAqIEMxIC0gQVsxXSAqIEMyICsgQVsyXSAqIEMzO1xufVxuXG5jb25zdCBtYXRyaXhJbnZlcnNlMzMgPSAoQSwgdGhyZXNob2xkKSA9PiB7XG4gIGNvbnN0IGRldCA9IGRldGVybWluYW50KEEpO1xuICBpZiAoTWF0aC5hYnMoZGV0KSA8PSB0aHJlc2hvbGQpIHJldHVybiBudWxsO1xuICBjb25zdCBvbmVPdmVyID0gMS4wIC8gZGV0O1xuXG4gIGNvbnN0IEIgPSBbXG4gICAgKEFbNF0gKiBBWzhdIC0gQVs1XSAqIEFbN10pICogb25lT3ZlcixcbiAgICAoQVsyXSAqIEFbN10gLSBBWzFdICogQVs4XSkgKiBvbmVPdmVyLFxuICAgIChBWzFdICogQVs1XSAtIEFbMl0gKiBBWzRdKSAqIG9uZU92ZXIsXG4gICAgKEFbNV0gKiBBWzZdIC0gQVszXSAqIEFbOF0pICogb25lT3ZlcixcbiAgICAoQVswXSAqIEFbOF0gLSBBWzJdICogQVs2XSkgKiBvbmVPdmVyLFxuICAgIChBWzJdICogQVszXSAtIEFbMF0gKiBBWzVdKSAqIG9uZU92ZXIsXG4gICAgKEFbM10gKiBBWzddIC0gQVs0XSAqIEFbNl0pICogb25lT3ZlcixcbiAgICAoQVsxXSAqIEFbNl0gLSBBWzBdICogQVs3XSkgKiBvbmVPdmVyLFxuICAgIChBWzBdICogQVs0XSAtIEFbMV0gKiBBWzNdKSAqIG9uZU92ZXIsXG4gIF07XG4gIHJldHVybiBCO1xufVxuXG5jb25zdCBtYXRyaXhNdWwzMyA9IChBLCBCKSA9PiB7XG4gIGNvbnN0IEMgPSBbXTtcbiAgQ1swXSA9IEFbMF0qQlswXSArIEFbMV0qQlszXSArIEFbMl0qQls2XTtcbiAgQ1sxXSA9IEFbMF0qQlsxXSArIEFbMV0qQls0XSArIEFbMl0qQls3XTtcbiAgQ1syXSA9IEFbMF0qQlsyXSArIEFbMV0qQls1XSArIEFbMl0qQls4XTtcbiAgQ1szXSA9IEFbM10qQlswXSArIEFbNF0qQlszXSArIEFbNV0qQls2XTtcbiAgQ1s0XSA9IEFbM10qQlsxXSArIEFbNF0qQls0XSArIEFbNV0qQls3XTtcbiAgQ1s1XSA9IEFbM10qQlsyXSArIEFbNF0qQls1XSArIEFbNV0qQls4XTtcbiAgQ1s2XSA9IEFbNl0qQlswXSArIEFbN10qQlszXSArIEFbOF0qQls2XTtcbiAgQ1s3XSA9IEFbNl0qQlsxXSArIEFbN10qQls0XSArIEFbOF0qQls3XTtcbiAgQ1s4XSA9IEFbNl0qQlsyXSArIEFbN10qQls1XSArIEFbOF0qQls4XTtcbiAgcmV0dXJuIEM7XG59XG5cbmNvbnN0IG11bHRpcGx5UG9pbnRIb21vZ3JhcGh5SW5ob21vZ2Vub3VzID0gKHgsIEgpID0+IHtcbiAgY29uc3QgdyA9IEhbNl0qeFswXSArIEhbN10qeFsxXSArIEhbOF07XG4gIGNvbnN0IHhwID0gW107XG4gIHhwWzBdID0gKEhbMF0qeFswXSArIEhbMV0qeFsxXSArIEhbMl0pL3c7XG4gIHhwWzFdID0gKEhbM10qeFswXSArIEhbNF0qeFsxXSArIEhbNV0pL3c7XG4gIHJldHVybiB4cDtcbn1cblxuY29uc3Qgc21hbGxlc3RUcmlhbmdsZUFyZWEgPSAoeDEsIHgyLCB4MywgeDQpID0+IHtcbiAgY29uc3QgdjEyID0gX3ZlY3Rvcih4MiwgeDEpO1xuICBjb25zdCB2MTMgPSBfdmVjdG9yKHgzLCB4MSk7XG4gIGNvbnN0IHYxNCA9IF92ZWN0b3IoeDQsIHgxKTtcbiAgY29uc3QgdjMyID0gX3ZlY3Rvcih4MiwgeDMpO1xuICBjb25zdCB2MzQgPSBfdmVjdG9yKHg0LCB4Myk7XG4gIGNvbnN0IGExID0gX2FyZWFPZlRyaWFuZ2xlKHYxMiwgdjEzKTtcbiAgY29uc3QgYTIgPSBfYXJlYU9mVHJpYW5nbGUodjEzLCB2MTQpO1xuICBjb25zdCBhMyA9IF9hcmVhT2ZUcmlhbmdsZSh2MTIsIHYxNCk7XG4gIGNvbnN0IGE0ID0gX2FyZWFPZlRyaWFuZ2xlKHYzMiwgdjM0KTtcbiAgcmV0dXJuIE1hdGgubWluKE1hdGgubWluKE1hdGgubWluKGExLCBhMiksIGEzKSwgYTQpO1xufVxuXG4vLyBjaGVjayBpZiBmb3VyIHBvaW50cyBmb3JtIGEgY29udmV4IHF1YWRyaWxhdGVybmFsLlxuLy8gYWxsIGZvdXIgY29tYmluYXRpb25zIHNob3VsZCBoYXZlIHNhbWUgc2lnblxuY29uc3QgcXVhZHJpbGF0ZXJhbENvbnZleCA9ICh4MSwgeDIsIHgzLCB4NCkgPT4ge1xuICBjb25zdCBmaXJzdCA9IGxpbmVQb2ludFNpZGUoeDEsIHgyLCB4MykgPD0gMDtcbiAgaWYgKCAobGluZVBvaW50U2lkZSh4MiwgeDMsIHg0KSA8PSAwKSAhPT0gZmlyc3QpIHJldHVybiBmYWxzZTtcbiAgaWYgKCAobGluZVBvaW50U2lkZSh4MywgeDQsIHgxKSA8PSAwKSAhPT0gZmlyc3QpIHJldHVybiBmYWxzZTtcbiAgaWYgKCAobGluZVBvaW50U2lkZSh4NCwgeDEsIHgyKSA8PSAwKSAhPT0gZmlyc3QpIHJldHVybiBmYWxzZTtcblxuICAvL2lmIChsaW5lUG9pbnRTaWRlKHgxLCB4MiwgeDMpIDw9IDApIHJldHVybiBmYWxzZTtcbiAgLy9pZiAobGluZVBvaW50U2lkZSh4MiwgeDMsIHg0KSA8PSAwKSByZXR1cm4gZmFsc2U7XG4gIC8vaWYgKGxpbmVQb2ludFNpZGUoeDMsIHg0LCB4MSkgPD0gMCkgcmV0dXJuIGZhbHNlO1xuICAvL2lmIChsaW5lUG9pbnRTaWRlKHg0LCB4MSwgeDIpIDw9IDApIHJldHVybiBmYWxzZTtcbiAgcmV0dXJuIHRydWU7XG59XG5cbmNvbnN0IF92ZWN0b3IgPSAoYSwgYikgPT4ge1xuICByZXR1cm4gW1xuICAgIGFbMF0gLSBiWzBdLFxuICAgIGFbMV0gLSBiWzFdXG4gIF1cbn1cblxuY29uc3QgX2FyZWFPZlRyaWFuZ2xlID0gKHUsIHYpID0+IHtcbiAgY29uc3QgYSA9IHVbMF0qdlsxXSAtIHVbMV0qdlswXTtcbiAgcmV0dXJuIE1hdGguYWJzKGEpICogMC41O1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IHtcbiAgbWF0cml4SW52ZXJzZTMzLFxuICBtYXRyaXhNdWwzMyxcbiAgcXVhZHJpbGF0ZXJhbENvbnZleCxcbiAgc21hbGxlc3RUcmlhbmdsZUFyZWEsXG4gIG11bHRpcGx5UG9pbnRIb21vZ3JhcGh5SW5ob21vZ2Vub3VzLFxuICBjaGVja1RocmVlUG9pbnRzQ29uc2lzdGVudCxcbiAgY2hlY2tGb3VyUG9pbnRzQ29uc2lzdGVudCxcbiAgZGV0ZXJtaW5hbnRcbn1cblxuIiwiY29uc3Qge01hdHJpeCwgaW52ZXJzZX0gPSByZXF1aXJlKCdtbC1tYXRyaXgnKTtcblxuY29uc3Qgc29sdmVIb21vZ3JhcGh5ID0gKHNyY1BvaW50cywgZHN0UG9pbnRzKSA9PiB7XG4gIGNvbnN0IHtub3JtUG9pbnRzOiBub3JtU3JjUG9pbnRzLCBwYXJhbTogc3JjUGFyYW19ID0gX25vcm1hbGl6ZVBvaW50cyhzcmNQb2ludHMpO1xuICBjb25zdCB7bm9ybVBvaW50czogbm9ybURzdFBvaW50cywgcGFyYW06IGRzdFBhcmFtfSA9IF9ub3JtYWxpemVQb2ludHMoZHN0UG9pbnRzKTtcblxuICBjb25zdCBudW0gPSBub3JtRHN0UG9pbnRzLmxlbmd0aDtcbiAgY29uc3QgQURhdGEgPSBbXTtcbiAgY29uc3QgQkRhdGEgPSBbXTtcbiAgZm9yIChsZXQgaiA9IDA7IGogPCBudW07IGorKykge1xuICAgIGNvbnN0IHJvdzEgPSBbXG4gICAgICBub3JtU3JjUG9pbnRzW2pdWzBdLFxuICAgICAgbm9ybVNyY1BvaW50c1tqXVsxXSxcbiAgICAgIDEsXG4gICAgICAwLFxuICAgICAgMCxcbiAgICAgIDAsXG4gICAgICAtKG5vcm1TcmNQb2ludHNbal1bMF0gKiBub3JtRHN0UG9pbnRzW2pdWzBdKSxcbiAgICAgIC0obm9ybVNyY1BvaW50c1tqXVsxXSAqIG5vcm1Ec3RQb2ludHNbal1bMF0pLFxuICAgIF07XG4gICAgY29uc3Qgcm93MiA9IFtcbiAgICAgIDAsXG4gICAgICAwLFxuICAgICAgMCxcbiAgICAgIG5vcm1TcmNQb2ludHNbal1bMF0sXG4gICAgICBub3JtU3JjUG9pbnRzW2pdWzFdLFxuICAgICAgMSxcbiAgICAgIC0obm9ybVNyY1BvaW50c1tqXVswXSAqIG5vcm1Ec3RQb2ludHNbal1bMV0pLFxuICAgICAgLShub3JtU3JjUG9pbnRzW2pdWzFdICogbm9ybURzdFBvaW50c1tqXVsxXSksXG4gICAgXTtcbiAgICBBRGF0YS5wdXNoKHJvdzEpO1xuICAgIEFEYXRhLnB1c2gocm93Mik7XG5cbiAgICBCRGF0YS5wdXNoKFtub3JtRHN0UG9pbnRzW2pdWzBdXSk7XG4gICAgQkRhdGEucHVzaChbbm9ybURzdFBvaW50c1tqXVsxXV0pO1xuICB9XG5cbiAgdHJ5IHtcbiAgICBjb25zdCBBID0gbmV3IE1hdHJpeChBRGF0YSk7XG4gICAgY29uc3QgQiA9IG5ldyBNYXRyaXgoQkRhdGEpO1xuICAgIGNvbnN0IEFUID0gQS50cmFuc3Bvc2UoKTtcbiAgICBjb25zdCBBVEEgPSBBVC5tbXVsKEEpO1xuICAgIGNvbnN0IEFUQiA9IEFULm1tdWwoQik7XG4gICAgY29uc3QgQVRBSW52ID0gaW52ZXJzZShBVEEpO1xuICAgIGNvbnN0IEMgPSBBVEFJbnYubW11bChBVEIpLnRvMURBcnJheSgpO1xuICAgIGNvbnN0IEggPSBfZGVub3JtYWxpemVIb21vZ3JhcGh5KEMsIHNyY1BhcmFtLCBkc3RQYXJhbSk7XG4gICAgcmV0dXJuIEg7XG4gIH0gY2F0Y2ggKGUpIHtcbiAgICByZXR1cm4gbnVsbDtcbiAgfVxufVxuXG4vLyBjZW50cm9pZCBhdCBvcmlnaW4gYW5kIGF2ZyBkaXN0YW5jZSBmcm9tIG9yaWdpbiBpcyBzcXJ0KDIpXG5jb25zdCBfbm9ybWFsaXplUG9pbnRzID0gKGNvb3JkcykgPT4ge1xuICAvL3JldHVybiB7bm9ybWFsaXplZENvb3JkczogY29vcmRzLCBwYXJhbToge21lYW5YOiAwLCBtZWFuWTogMCwgczogMX19OyAvLyBza2lwIG5vcm1hbGl6YXRpb25cblxuICBsZXQgc3VtWCA9IDA7XG4gIGxldCBzdW1ZID0gMDtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBjb29yZHMubGVuZ3RoOyBpKyspIHtcbiAgICBzdW1YICs9IGNvb3Jkc1tpXVswXTtcbiAgICBzdW1ZICs9IGNvb3Jkc1tpXVsxXTtcbiAgfVxuICBsZXQgbWVhblggPSBzdW1YIC8gY29vcmRzLmxlbmd0aDtcbiAgbGV0IG1lYW5ZID0gc3VtWSAvIGNvb3Jkcy5sZW5ndGg7XG5cbiAgbGV0IHN1bURpZmYgPSAwO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGNvb3Jkcy5sZW5ndGg7IGkrKykge1xuICAgIGNvbnN0IGRpZmZYID0gY29vcmRzW2ldWzBdIC0gbWVhblg7XG4gICAgY29uc3QgZGlmZlkgPSBjb29yZHNbaV1bMV0gLSBtZWFuWTtcbiAgICBzdW1EaWZmICs9IE1hdGguc3FydChkaWZmWCAqIGRpZmZYICsgZGlmZlkgKiBkaWZmWSk7XG4gIH1cbiAgbGV0IHMgPSBNYXRoLnNxcnQoMikgKiBjb29yZHMubGVuZ3RoIC8gc3VtRGlmZjtcblxuICBjb25zdCBub3JtUG9pbnRzID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgY29vcmRzLmxlbmd0aDsgaSsrKSB7XG4gICAgbm9ybVBvaW50cy5wdXNoKFtcbiAgICAgIChjb29yZHNbaV1bMF0gLSBtZWFuWCkgKiBzLFxuICAgICAgKGNvb3Jkc1tpXVsxXSAtIG1lYW5ZKSAqIHMsXG4gICAgXSk7XG4gIH1cbiAgcmV0dXJuIHtub3JtUG9pbnRzLCBwYXJhbToge21lYW5YLCBtZWFuWSwgc319O1xufVxuXG4vLyBEZW5vcm1hbGl6ZSBob21vZ3JhcGh5XG4vLyB3aGVyZSBUIGlzIHRoZSBub3JtYWxpemF0aW9uIG1hdHJpeCwgaS5lLlxuLy9cbi8vICAgICBbMSAgMCAgLW1lYW5YXVxuLy8gVCA9IFswICAxICAtbWVhblldXG4vLyAgICAgWzAgIDAgICAgIDEvc11cbi8vXG4vLyAgICAgICAgICBbMSAgMCAgcyptZWFuWF1cbi8vIGludihUKSA9IFswICAxICBzKm1lYW5ZXVxuLy8gXHQgICAgWzAgIDAgICAgICAgIHNdXG4vL1xuLy8gSCA9IGludihUZHN0KSAqIEhuICogVHNyY1xuLy9cbi8vIEBwYXJhbSB7XG4vLyAgIG5IOiBub3JtSCxcbi8vICAgc3JjUGFyYW06IHBhcmFtIG9mIHNyYyB0cmFuc2Zvcm0sXG4vLyAgIGRzdFBhcmFtOiBwYXJhbSBvZiBkc3QgdHJhbnNmb3JtXG4vLyB9XG5jb25zdCBfZGVub3JtYWxpemVIb21vZ3JhcGh5ID0gKG5ILCBzcmNQYXJhbSwgZHN0UGFyYW0pID0+IHtcbiAgLypcbiAgTWF0cml4IHZlcnNpb25cbiAgY29uc3Qgbm9ybUggPSBuZXcgTWF0cml4KFtcbiAgICBbbkhbMF0sIG5IWzFdLCBuSFsyXV0sXG4gICAgW25IWzNdLCBuSFs0XSwgbkhbNV1dLFxuICAgIFtuSFs2XSwgbkhbN10sIDFdLFxuICBdKTtcbiAgY29uc3QgVHNyYyA9IG5ldyBNYXRyaXgoW1xuICAgIFsxLCAwLCAtc3JjUGFyYW0ubWVhblhdLFxuICAgIFswLCAxLCAtc3JjUGFyYW0ubWVhblldLFxuICAgIFswLCAwLCAgICAxL3NyY1BhcmFtLnNdLFxuICBdKTtcblxuICBjb25zdCBpbnZUZHN0ID0gbmV3IE1hdHJpeChbXG4gICAgWzEsIDAsIGRzdFBhcmFtLnMgKiBkc3RQYXJhbS5tZWFuWF0sXG4gICAgWzAsIDEsIGRzdFBhcmFtLnMgKiBkc3RQYXJhbS5tZWFuWV0sXG4gICAgWzAsIDAsIGRzdFBhcmFtLnNdLFxuICBdKTtcbiAgY29uc3QgSCA9IGludlRkc3QubW11bChub3JtSCkubW11bChUc3JjKTtcbiAgKi9cblxuICAvLyBwbGFpbiBpbXBsZW1lbnRhdGlvbiBvZiB0aGUgYWJvdmUgdXNpbmcgTWF0cml4XG4gIGNvbnN0IHNNZWFuWCA9IGRzdFBhcmFtLnMgKiBkc3RQYXJhbS5tZWFuWDtcbiAgY29uc3Qgc01lYW5ZID0gZHN0UGFyYW0ucyAqIGRzdFBhcmFtLm1lYW5ZO1xuXG4gIGNvbnN0IEggPSBbXG4gICAgICBuSFswXSArIHNNZWFuWCAqIG5IWzZdLCBcbiAgICAgIG5IWzFdICsgc01lYW5YICogbkhbN10sXG4gICAgICAobkhbMF0gKyBzTWVhblggKiBuSFs2XSkgKiAtc3JjUGFyYW0ubWVhblggKyAobkhbMV0gKyBzTWVhblggKiBuSFs3XSkgKiAtc3JjUGFyYW0ubWVhblkgKyAobkhbMl0gKyBzTWVhblgpIC8gc3JjUGFyYW0ucyxcbiAgICAgIG5IWzNdICsgc01lYW5ZICogbkhbNl0sIFxuICAgICAgbkhbNF0gKyBzTWVhblkgKiBuSFs3XSxcbiAgICAgIChuSFszXSArIHNNZWFuWSAqIG5IWzZdKSAqIC1zcmNQYXJhbS5tZWFuWCArIChuSFs0XSArIHNNZWFuWSAqIG5IWzddKSAqIC1zcmNQYXJhbS5tZWFuWSArIChuSFs1XSArIHNNZWFuWSkgLyBzcmNQYXJhbS5zLFxuICAgICAgZHN0UGFyYW0ucyAqIG5IWzZdLFxuICAgICAgZHN0UGFyYW0ucyAqIG5IWzddLFxuICAgICAgZHN0UGFyYW0ucyAqIG5IWzZdICogLXNyY1BhcmFtLm1lYW5YICsgZHN0UGFyYW0ucyAqIG5IWzddICogLXNyY1BhcmFtLm1lYW5ZICsgZHN0UGFyYW0ucyAvIHNyY1BhcmFtLnMsXG4gIF07XG5cbiAgLy8gbWFrZSBIWzhdID09PSAxO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IDk7IGkrKykge1xuICAgIEhbaV0gPSBIW2ldIC8gSFs4XTtcbiAgfVxuICByZXR1cm4gSDtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSB7XG4gIHNvbHZlSG9tb2dyYXBoeVxufVxuIiwiY29uc3QgbVJhbmRTZWVkID0gMTIzNDtcblxuY29uc3QgY3JlYXRlUmFuZG9taXplciA9ICgpID0+IHtcbiAgY29uc3QgcmFuZG9taXplciA9IHtcbiAgICBzZWVkOiBtUmFuZFNlZWQsXG5cbiAgICBhcnJheVNodWZmbGUob3B0aW9ucykge1xuICAgICAgY29uc3Qge2Fyciwgc2FtcGxlU2l6ZX0gPSBvcHRpb25zO1xuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCBzYW1wbGVTaXplOyBpKyspIHtcblxuICAgICAgICB0aGlzLnNlZWQgPSAoMjE0MDEzICogdGhpcy5zZWVkICsgMjUzMTAxMSkgJSAoMSA8PCAzMSk7XG4gICAgICAgIGxldCBrID0gKHRoaXMuc2VlZCA+PiAxNikgJiAweDdmZmY7XG4gICAgICAgIGsgPSBrICUgYXJyLmxlbmd0aDtcblxuICAgICAgICBsZXQgdG1wID0gYXJyW2ldO1xuICAgICAgICBhcnJbaV0gPSBhcnJba107XG4gICAgICAgIGFycltrXSA9IHRtcDtcbiAgICAgIH1cbiAgICB9LFxuXG4gICAgbmV4dEludChtYXhWYWx1ZSkge1xuICAgICAgdGhpcy5zZWVkID0gKDIxNDAxMyAqIHRoaXMuc2VlZCArIDI1MzEwMTEpICUgKDEgPDwgMzEpO1xuICAgICAgbGV0IGsgPSAodGhpcy5zZWVkID4+IDE2KSAmIDB4N2ZmZjtcbiAgICAgIGsgPSBrICUgbWF4VmFsdWU7XG4gICAgICByZXR1cm4gaztcbiAgICB9XG4gIH1cbiAgcmV0dXJuIHJhbmRvbWl6ZXI7XG59XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuICBjcmVhdGVSYW5kb21pemVyXG59XG4iLCJjb25zdCB0b1N0cmluZyA9IE9iamVjdC5wcm90b3R5cGUudG9TdHJpbmc7XG4vKipcbiAqIENoZWNrcyBpZiBhbiBvYmplY3QgaXMgYW4gaW5zdGFuY2Ugb2YgYW4gQXJyYXkgKGFycmF5IG9yIHR5cGVkIGFycmF5KS5cbiAqXG4gKiBAcGFyYW0ge2FueX0gdmFsdWUgLSBPYmplY3QgdG8gY2hlY2suXG4gKiBAcmV0dXJucyB7Ym9vbGVhbn0gVHJ1ZSBpZiB0aGUgb2JqZWN0IGlzIGFuIGFycmF5LlxuICovXG5leHBvcnQgZnVuY3Rpb24gaXNBbnlBcnJheSh2YWx1ZSkge1xuICAgIHJldHVybiB0b1N0cmluZy5jYWxsKHZhbHVlKS5lbmRzV2l0aCgnQXJyYXldJyk7XG59XG4vLyMgc291cmNlTWFwcGluZ1VSTD1pbmRleC5qcy5tYXAiLCJpbXBvcnQgeyBpc0FueUFycmF5IH0gZnJvbSAnaXMtYW55LWFycmF5JztcblxuZnVuY3Rpb24gbWF4KGlucHV0KSB7XG4gIHZhciBvcHRpb25zID0gYXJndW1lbnRzLmxlbmd0aCA+IDEgJiYgYXJndW1lbnRzWzFdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMV0gOiB7fTtcblxuICBpZiAoIWlzQW55QXJyYXkoaW5wdXQpKSB7XG4gICAgdGhyb3cgbmV3IFR5cGVFcnJvcignaW5wdXQgbXVzdCBiZSBhbiBhcnJheScpO1xuICB9XG5cbiAgaWYgKGlucHV0Lmxlbmd0aCA9PT0gMCkge1xuICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ2lucHV0IG11c3Qgbm90IGJlIGVtcHR5Jyk7XG4gIH1cblxuICB2YXIgX29wdGlvbnMkZnJvbUluZGV4ID0gb3B0aW9ucy5mcm9tSW5kZXgsXG4gICAgICBmcm9tSW5kZXggPSBfb3B0aW9ucyRmcm9tSW5kZXggPT09IHZvaWQgMCA/IDAgOiBfb3B0aW9ucyRmcm9tSW5kZXgsXG4gICAgICBfb3B0aW9ucyR0b0luZGV4ID0gb3B0aW9ucy50b0luZGV4LFxuICAgICAgdG9JbmRleCA9IF9vcHRpb25zJHRvSW5kZXggPT09IHZvaWQgMCA/IGlucHV0Lmxlbmd0aCA6IF9vcHRpb25zJHRvSW5kZXg7XG5cbiAgaWYgKGZyb21JbmRleCA8IDAgfHwgZnJvbUluZGV4ID49IGlucHV0Lmxlbmd0aCB8fCAhTnVtYmVyLmlzSW50ZWdlcihmcm9tSW5kZXgpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKCdmcm9tSW5kZXggbXVzdCBiZSBhIHBvc2l0aXZlIGludGVnZXIgc21hbGxlciB0aGFuIGxlbmd0aCcpO1xuICB9XG5cbiAgaWYgKHRvSW5kZXggPD0gZnJvbUluZGV4IHx8IHRvSW5kZXggPiBpbnB1dC5sZW5ndGggfHwgIU51bWJlci5pc0ludGVnZXIodG9JbmRleCkpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoJ3RvSW5kZXggbXVzdCBiZSBhbiBpbnRlZ2VyIGdyZWF0ZXIgdGhhbiBmcm9tSW5kZXggYW5kIGF0IG1vc3QgZXF1YWwgdG8gbGVuZ3RoJyk7XG4gIH1cblxuICB2YXIgbWF4VmFsdWUgPSBpbnB1dFtmcm9tSW5kZXhdO1xuXG4gIGZvciAodmFyIGkgPSBmcm9tSW5kZXggKyAxOyBpIDwgdG9JbmRleDsgaSsrKSB7XG4gICAgaWYgKGlucHV0W2ldID4gbWF4VmFsdWUpIG1heFZhbHVlID0gaW5wdXRbaV07XG4gIH1cblxuICByZXR1cm4gbWF4VmFsdWU7XG59XG5cbmV4cG9ydCB7IG1heCBhcyBkZWZhdWx0IH07XG4iLCJpbXBvcnQgeyBpc0FueUFycmF5IH0gZnJvbSAnaXMtYW55LWFycmF5JztcblxuZnVuY3Rpb24gbWluKGlucHV0KSB7XG4gIHZhciBvcHRpb25zID0gYXJndW1lbnRzLmxlbmd0aCA+IDEgJiYgYXJndW1lbnRzWzFdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMV0gOiB7fTtcblxuICBpZiAoIWlzQW55QXJyYXkoaW5wdXQpKSB7XG4gICAgdGhyb3cgbmV3IFR5cGVFcnJvcignaW5wdXQgbXVzdCBiZSBhbiBhcnJheScpO1xuICB9XG5cbiAgaWYgKGlucHV0Lmxlbmd0aCA9PT0gMCkge1xuICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ2lucHV0IG11c3Qgbm90IGJlIGVtcHR5Jyk7XG4gIH1cblxuICB2YXIgX29wdGlvbnMkZnJvbUluZGV4ID0gb3B0aW9ucy5mcm9tSW5kZXgsXG4gICAgICBmcm9tSW5kZXggPSBfb3B0aW9ucyRmcm9tSW5kZXggPT09IHZvaWQgMCA/IDAgOiBfb3B0aW9ucyRmcm9tSW5kZXgsXG4gICAgICBfb3B0aW9ucyR0b0luZGV4ID0gb3B0aW9ucy50b0luZGV4LFxuICAgICAgdG9JbmRleCA9IF9vcHRpb25zJHRvSW5kZXggPT09IHZvaWQgMCA/IGlucHV0Lmxlbmd0aCA6IF9vcHRpb25zJHRvSW5kZXg7XG5cbiAgaWYgKGZyb21JbmRleCA8IDAgfHwgZnJvbUluZGV4ID49IGlucHV0Lmxlbmd0aCB8fCAhTnVtYmVyLmlzSW50ZWdlcihmcm9tSW5kZXgpKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKCdmcm9tSW5kZXggbXVzdCBiZSBhIHBvc2l0aXZlIGludGVnZXIgc21hbGxlciB0aGFuIGxlbmd0aCcpO1xuICB9XG5cbiAgaWYgKHRvSW5kZXggPD0gZnJvbUluZGV4IHx8IHRvSW5kZXggPiBpbnB1dC5sZW5ndGggfHwgIU51bWJlci5pc0ludGVnZXIodG9JbmRleCkpIHtcbiAgICB0aHJvdyBuZXcgRXJyb3IoJ3RvSW5kZXggbXVzdCBiZSBhbiBpbnRlZ2VyIGdyZWF0ZXIgdGhhbiBmcm9tSW5kZXggYW5kIGF0IG1vc3QgZXF1YWwgdG8gbGVuZ3RoJyk7XG4gIH1cblxuICB2YXIgbWluVmFsdWUgPSBpbnB1dFtmcm9tSW5kZXhdO1xuXG4gIGZvciAodmFyIGkgPSBmcm9tSW5kZXggKyAxOyBpIDwgdG9JbmRleDsgaSsrKSB7XG4gICAgaWYgKGlucHV0W2ldIDwgbWluVmFsdWUpIG1pblZhbHVlID0gaW5wdXRbaV07XG4gIH1cblxuICByZXR1cm4gbWluVmFsdWU7XG59XG5cbmV4cG9ydCB7IG1pbiBhcyBkZWZhdWx0IH07XG4iLCJpbXBvcnQgeyBpc0FueUFycmF5IH0gZnJvbSAnaXMtYW55LWFycmF5JztcbmltcG9ydCBtYXggZnJvbSAnbWwtYXJyYXktbWF4JztcbmltcG9ydCBtaW4gZnJvbSAnbWwtYXJyYXktbWluJztcblxuZnVuY3Rpb24gcmVzY2FsZShpbnB1dCkge1xuICB2YXIgb3B0aW9ucyA9IGFyZ3VtZW50cy5sZW5ndGggPiAxICYmIGFyZ3VtZW50c1sxXSAhPT0gdW5kZWZpbmVkID8gYXJndW1lbnRzWzFdIDoge307XG5cbiAgaWYgKCFpc0FueUFycmF5KGlucHV0KSkge1xuICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ2lucHV0IG11c3QgYmUgYW4gYXJyYXknKTtcbiAgfSBlbHNlIGlmIChpbnB1dC5sZW5ndGggPT09IDApIHtcbiAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdpbnB1dCBtdXN0IG5vdCBiZSBlbXB0eScpO1xuICB9XG5cbiAgdmFyIG91dHB1dDtcblxuICBpZiAob3B0aW9ucy5vdXRwdXQgIT09IHVuZGVmaW5lZCkge1xuICAgIGlmICghaXNBbnlBcnJheShvcHRpb25zLm91dHB1dCkpIHtcbiAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ291dHB1dCBvcHRpb24gbXVzdCBiZSBhbiBhcnJheSBpZiBzcGVjaWZpZWQnKTtcbiAgICB9XG5cbiAgICBvdXRwdXQgPSBvcHRpb25zLm91dHB1dDtcbiAgfSBlbHNlIHtcbiAgICBvdXRwdXQgPSBuZXcgQXJyYXkoaW5wdXQubGVuZ3RoKTtcbiAgfVxuXG4gIHZhciBjdXJyZW50TWluID0gbWluKGlucHV0KTtcbiAgdmFyIGN1cnJlbnRNYXggPSBtYXgoaW5wdXQpO1xuXG4gIGlmIChjdXJyZW50TWluID09PSBjdXJyZW50TWF4KSB7XG4gICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ21pbmltdW0gYW5kIG1heGltdW0gaW5wdXQgdmFsdWVzIGFyZSBlcXVhbC4gQ2Fubm90IHJlc2NhbGUgYSBjb25zdGFudCBhcnJheScpO1xuICB9XG5cbiAgdmFyIF9vcHRpb25zJG1pbiA9IG9wdGlvbnMubWluLFxuICAgICAgbWluVmFsdWUgPSBfb3B0aW9ucyRtaW4gPT09IHZvaWQgMCA/IG9wdGlvbnMuYXV0b01pbk1heCA/IGN1cnJlbnRNaW4gOiAwIDogX29wdGlvbnMkbWluLFxuICAgICAgX29wdGlvbnMkbWF4ID0gb3B0aW9ucy5tYXgsXG4gICAgICBtYXhWYWx1ZSA9IF9vcHRpb25zJG1heCA9PT0gdm9pZCAwID8gb3B0aW9ucy5hdXRvTWluTWF4ID8gY3VycmVudE1heCA6IDEgOiBfb3B0aW9ucyRtYXg7XG5cbiAgaWYgKG1pblZhbHVlID49IG1heFZhbHVlKSB7XG4gICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ21pbiBvcHRpb24gbXVzdCBiZSBzbWFsbGVyIHRoYW4gbWF4IG9wdGlvbicpO1xuICB9XG5cbiAgdmFyIGZhY3RvciA9IChtYXhWYWx1ZSAtIG1pblZhbHVlKSAvIChjdXJyZW50TWF4IC0gY3VycmVudE1pbik7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBpbnB1dC5sZW5ndGg7IGkrKykge1xuICAgIG91dHB1dFtpXSA9IChpbnB1dFtpXSAtIGN1cnJlbnRNaW4pICogZmFjdG9yICsgbWluVmFsdWU7XG4gIH1cblxuICByZXR1cm4gb3V0cHV0O1xufVxuXG5leHBvcnQgeyByZXNjYWxlIGFzIGRlZmF1bHQgfTtcbiIsImltcG9ydCB7IGlzQW55QXJyYXkgfSBmcm9tICdpcy1hbnktYXJyYXknO1xuXG5pbXBvcnQgTWF0cml4IGZyb20gJy4vbWF0cml4JztcblxuZXhwb3J0IGZ1bmN0aW9uIGNvcnJlbGF0aW9uKHhNYXRyaXgsIHlNYXRyaXggPSB4TWF0cml4LCBvcHRpb25zID0ge30pIHtcbiAgeE1hdHJpeCA9IG5ldyBNYXRyaXgoeE1hdHJpeCk7XG4gIGxldCB5SXNTYW1lID0gZmFsc2U7XG4gIGlmIChcbiAgICB0eXBlb2YgeU1hdHJpeCA9PT0gJ29iamVjdCcgJiZcbiAgICAhTWF0cml4LmlzTWF0cml4KHlNYXRyaXgpICYmXG4gICAgIWlzQW55QXJyYXkoeU1hdHJpeClcbiAgKSB7XG4gICAgb3B0aW9ucyA9IHlNYXRyaXg7XG4gICAgeU1hdHJpeCA9IHhNYXRyaXg7XG4gICAgeUlzU2FtZSA9IHRydWU7XG4gIH0gZWxzZSB7XG4gICAgeU1hdHJpeCA9IG5ldyBNYXRyaXgoeU1hdHJpeCk7XG4gIH1cbiAgaWYgKHhNYXRyaXgucm93cyAhPT0geU1hdHJpeC5yb3dzKSB7XG4gICAgdGhyb3cgbmV3IFR5cGVFcnJvcignQm90aCBtYXRyaWNlcyBtdXN0IGhhdmUgdGhlIHNhbWUgbnVtYmVyIG9mIHJvd3MnKTtcbiAgfVxuXG4gIGNvbnN0IHsgY2VudGVyID0gdHJ1ZSwgc2NhbGUgPSB0cnVlIH0gPSBvcHRpb25zO1xuICBpZiAoY2VudGVyKSB7XG4gICAgeE1hdHJpeC5jZW50ZXIoJ2NvbHVtbicpO1xuICAgIGlmICgheUlzU2FtZSkge1xuICAgICAgeU1hdHJpeC5jZW50ZXIoJ2NvbHVtbicpO1xuICAgIH1cbiAgfVxuICBpZiAoc2NhbGUpIHtcbiAgICB4TWF0cml4LnNjYWxlKCdjb2x1bW4nKTtcbiAgICBpZiAoIXlJc1NhbWUpIHtcbiAgICAgIHlNYXRyaXguc2NhbGUoJ2NvbHVtbicpO1xuICAgIH1cbiAgfVxuXG4gIGNvbnN0IHNkeCA9IHhNYXRyaXguc3RhbmRhcmREZXZpYXRpb24oJ2NvbHVtbicsIHsgdW5iaWFzZWQ6IHRydWUgfSk7XG4gIGNvbnN0IHNkeSA9IHlJc1NhbWVcbiAgICA/IHNkeFxuICAgIDogeU1hdHJpeC5zdGFuZGFyZERldmlhdGlvbignY29sdW1uJywgeyB1bmJpYXNlZDogdHJ1ZSB9KTtcblxuICBjb25zdCBjb3JyID0geE1hdHJpeC50cmFuc3Bvc2UoKS5tbXVsKHlNYXRyaXgpO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGNvcnIucm93czsgaSsrKSB7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBjb3JyLmNvbHVtbnM7IGorKykge1xuICAgICAgY29yci5zZXQoXG4gICAgICAgIGksXG4gICAgICAgIGosXG4gICAgICAgIGNvcnIuZ2V0KGksIGopICogKDEgLyAoc2R4W2ldICogc2R5W2pdKSkgKiAoMSAvICh4TWF0cml4LnJvd3MgLSAxKSksXG4gICAgICApO1xuICAgIH1cbiAgfVxuICByZXR1cm4gY29ycjtcbn1cbiIsImltcG9ydCB7IGlzQW55QXJyYXkgfSBmcm9tICdpcy1hbnktYXJyYXknO1xuXG5pbXBvcnQgTWF0cml4IGZyb20gJy4vbWF0cml4JztcblxuZXhwb3J0IGZ1bmN0aW9uIGNvdmFyaWFuY2UoeE1hdHJpeCwgeU1hdHJpeCA9IHhNYXRyaXgsIG9wdGlvbnMgPSB7fSkge1xuICB4TWF0cml4ID0gbmV3IE1hdHJpeCh4TWF0cml4KTtcbiAgbGV0IHlJc1NhbWUgPSBmYWxzZTtcbiAgaWYgKFxuICAgIHR5cGVvZiB5TWF0cml4ID09PSAnb2JqZWN0JyAmJlxuICAgICFNYXRyaXguaXNNYXRyaXgoeU1hdHJpeCkgJiZcbiAgICAhaXNBbnlBcnJheSh5TWF0cml4KVxuICApIHtcbiAgICBvcHRpb25zID0geU1hdHJpeDtcbiAgICB5TWF0cml4ID0geE1hdHJpeDtcbiAgICB5SXNTYW1lID0gdHJ1ZTtcbiAgfSBlbHNlIHtcbiAgICB5TWF0cml4ID0gbmV3IE1hdHJpeCh5TWF0cml4KTtcbiAgfVxuICBpZiAoeE1hdHJpeC5yb3dzICE9PSB5TWF0cml4LnJvd3MpIHtcbiAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdCb3RoIG1hdHJpY2VzIG11c3QgaGF2ZSB0aGUgc2FtZSBudW1iZXIgb2Ygcm93cycpO1xuICB9XG4gIGNvbnN0IHsgY2VudGVyID0gdHJ1ZSB9ID0gb3B0aW9ucztcbiAgaWYgKGNlbnRlcikge1xuICAgIHhNYXRyaXggPSB4TWF0cml4LmNlbnRlcignY29sdW1uJyk7XG4gICAgaWYgKCF5SXNTYW1lKSB7XG4gICAgICB5TWF0cml4ID0geU1hdHJpeC5jZW50ZXIoJ2NvbHVtbicpO1xuICAgIH1cbiAgfVxuICBjb25zdCBjb3YgPSB4TWF0cml4LnRyYW5zcG9zZSgpLm1tdWwoeU1hdHJpeCk7XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgY292LnJvd3M7IGkrKykge1xuICAgIGZvciAobGV0IGogPSAwOyBqIDwgY292LmNvbHVtbnM7IGorKykge1xuICAgICAgY292LnNldChpLCBqLCBjb3YuZ2V0KGksIGopICogKDEgLyAoeE1hdHJpeC5yb3dzIC0gMSkpKTtcbiAgICB9XG4gIH1cbiAgcmV0dXJuIGNvdjtcbn1cbiIsImltcG9ydCBNYXRyaXggZnJvbSAnLi4vbWF0cml4JztcbmltcG9ydCBXcmFwcGVyTWF0cml4MkQgZnJvbSAnLi4vd3JhcC9XcmFwcGVyTWF0cml4MkQnO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBDaG9sZXNreURlY29tcG9zaXRpb24ge1xuICBjb25zdHJ1Y3Rvcih2YWx1ZSkge1xuICAgIHZhbHVlID0gV3JhcHBlck1hdHJpeDJELmNoZWNrTWF0cml4KHZhbHVlKTtcbiAgICBpZiAoIXZhbHVlLmlzU3ltbWV0cmljKCkpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcignTWF0cml4IGlzIG5vdCBzeW1tZXRyaWMnKTtcbiAgICB9XG5cbiAgICBsZXQgYSA9IHZhbHVlO1xuICAgIGxldCBkaW1lbnNpb24gPSBhLnJvd3M7XG4gICAgbGV0IGwgPSBuZXcgTWF0cml4KGRpbWVuc2lvbiwgZGltZW5zaW9uKTtcbiAgICBsZXQgcG9zaXRpdmVEZWZpbml0ZSA9IHRydWU7XG4gICAgbGV0IGksIGosIGs7XG5cbiAgICBmb3IgKGogPSAwOyBqIDwgZGltZW5zaW9uOyBqKyspIHtcbiAgICAgIGxldCBkID0gMDtcbiAgICAgIGZvciAoayA9IDA7IGsgPCBqOyBrKyspIHtcbiAgICAgICAgbGV0IHMgPSAwO1xuICAgICAgICBmb3IgKGkgPSAwOyBpIDwgazsgaSsrKSB7XG4gICAgICAgICAgcyArPSBsLmdldChrLCBpKSAqIGwuZ2V0KGosIGkpO1xuICAgICAgICB9XG4gICAgICAgIHMgPSAoYS5nZXQoaiwgaykgLSBzKSAvIGwuZ2V0KGssIGspO1xuICAgICAgICBsLnNldChqLCBrLCBzKTtcbiAgICAgICAgZCA9IGQgKyBzICogcztcbiAgICAgIH1cblxuICAgICAgZCA9IGEuZ2V0KGosIGopIC0gZDtcblxuICAgICAgcG9zaXRpdmVEZWZpbml0ZSAmPSBkID4gMDtcbiAgICAgIGwuc2V0KGosIGosIE1hdGguc3FydChNYXRoLm1heChkLCAwKSkpO1xuICAgICAgZm9yIChrID0gaiArIDE7IGsgPCBkaW1lbnNpb247IGsrKykge1xuICAgICAgICBsLnNldChqLCBrLCAwKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICB0aGlzLkwgPSBsO1xuICAgIHRoaXMucG9zaXRpdmVEZWZpbml0ZSA9IEJvb2xlYW4ocG9zaXRpdmVEZWZpbml0ZSk7XG4gIH1cblxuICBpc1Bvc2l0aXZlRGVmaW5pdGUoKSB7XG4gICAgcmV0dXJuIHRoaXMucG9zaXRpdmVEZWZpbml0ZTtcbiAgfVxuXG4gIHNvbHZlKHZhbHVlKSB7XG4gICAgdmFsdWUgPSBXcmFwcGVyTWF0cml4MkQuY2hlY2tNYXRyaXgodmFsdWUpO1xuXG4gICAgbGV0IGwgPSB0aGlzLkw7XG4gICAgbGV0IGRpbWVuc2lvbiA9IGwucm93cztcblxuICAgIGlmICh2YWx1ZS5yb3dzICE9PSBkaW1lbnNpb24pIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcignTWF0cml4IGRpbWVuc2lvbnMgZG8gbm90IG1hdGNoJyk7XG4gICAgfVxuICAgIGlmICh0aGlzLmlzUG9zaXRpdmVEZWZpbml0ZSgpID09PSBmYWxzZSkge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKCdNYXRyaXggaXMgbm90IHBvc2l0aXZlIGRlZmluaXRlJyk7XG4gICAgfVxuXG4gICAgbGV0IGNvdW50ID0gdmFsdWUuY29sdW1ucztcbiAgICBsZXQgQiA9IHZhbHVlLmNsb25lKCk7XG4gICAgbGV0IGksIGosIGs7XG5cbiAgICBmb3IgKGsgPSAwOyBrIDwgZGltZW5zaW9uOyBrKyspIHtcbiAgICAgIGZvciAoaiA9IDA7IGogPCBjb3VudDsgaisrKSB7XG4gICAgICAgIGZvciAoaSA9IDA7IGkgPCBrOyBpKyspIHtcbiAgICAgICAgICBCLnNldChrLCBqLCBCLmdldChrLCBqKSAtIEIuZ2V0KGksIGopICogbC5nZXQoaywgaSkpO1xuICAgICAgICB9XG4gICAgICAgIEIuc2V0KGssIGosIEIuZ2V0KGssIGopIC8gbC5nZXQoaywgaykpO1xuICAgICAgfVxuICAgIH1cblxuICAgIGZvciAoayA9IGRpbWVuc2lvbiAtIDE7IGsgPj0gMDsgay0tKSB7XG4gICAgICBmb3IgKGogPSAwOyBqIDwgY291bnQ7IGorKykge1xuICAgICAgICBmb3IgKGkgPSBrICsgMTsgaSA8IGRpbWVuc2lvbjsgaSsrKSB7XG4gICAgICAgICAgQi5zZXQoaywgaiwgQi5nZXQoaywgaikgLSBCLmdldChpLCBqKSAqIGwuZ2V0KGksIGspKTtcbiAgICAgICAgfVxuICAgICAgICBCLnNldChrLCBqLCBCLmdldChrLCBqKSAvIGwuZ2V0KGssIGspKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICByZXR1cm4gQjtcbiAgfVxuXG4gIGdldCBsb3dlclRyaWFuZ3VsYXJNYXRyaXgoKSB7XG4gICAgcmV0dXJuIHRoaXMuTDtcbiAgfVxufVxuIiwiaW1wb3J0IE1hdHJpeCBmcm9tICcuLi9tYXRyaXgnO1xuaW1wb3J0IFdyYXBwZXJNYXRyaXgyRCBmcm9tICcuLi93cmFwL1dyYXBwZXJNYXRyaXgyRCc7XG5cbmltcG9ydCB7IGh5cG90ZW51c2UgfSBmcm9tICcuL3V0aWwnO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBFaWdlbnZhbHVlRGVjb21wb3NpdGlvbiB7XG4gIGNvbnN0cnVjdG9yKG1hdHJpeCwgb3B0aW9ucyA9IHt9KSB7XG4gICAgY29uc3QgeyBhc3N1bWVTeW1tZXRyaWMgPSBmYWxzZSB9ID0gb3B0aW9ucztcblxuICAgIG1hdHJpeCA9IFdyYXBwZXJNYXRyaXgyRC5jaGVja01hdHJpeChtYXRyaXgpO1xuICAgIGlmICghbWF0cml4LmlzU3F1YXJlKCkpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcignTWF0cml4IGlzIG5vdCBhIHNxdWFyZSBtYXRyaXgnKTtcbiAgICB9XG5cbiAgICBpZiAobWF0cml4LmlzRW1wdHkoKSkge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKCdNYXRyaXggbXVzdCBiZSBub24tZW1wdHknKTtcbiAgICB9XG5cbiAgICBsZXQgbiA9IG1hdHJpeC5jb2x1bW5zO1xuICAgIGxldCBWID0gbmV3IE1hdHJpeChuLCBuKTtcbiAgICBsZXQgZCA9IG5ldyBGbG9hdDY0QXJyYXkobik7XG4gICAgbGV0IGUgPSBuZXcgRmxvYXQ2NEFycmF5KG4pO1xuICAgIGxldCB2YWx1ZSA9IG1hdHJpeDtcbiAgICBsZXQgaSwgajtcblxuICAgIGxldCBpc1N5bW1ldHJpYyA9IGZhbHNlO1xuICAgIGlmIChhc3N1bWVTeW1tZXRyaWMpIHtcbiAgICAgIGlzU3ltbWV0cmljID0gdHJ1ZTtcbiAgICB9IGVsc2Uge1xuICAgICAgaXNTeW1tZXRyaWMgPSBtYXRyaXguaXNTeW1tZXRyaWMoKTtcbiAgICB9XG5cbiAgICBpZiAoaXNTeW1tZXRyaWMpIHtcbiAgICAgIGZvciAoaSA9IDA7IGkgPCBuOyBpKyspIHtcbiAgICAgICAgZm9yIChqID0gMDsgaiA8IG47IGorKykge1xuICAgICAgICAgIFYuc2V0KGksIGosIHZhbHVlLmdldChpLCBqKSk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIHRyZWQyKG4sIGUsIGQsIFYpO1xuICAgICAgdHFsMihuLCBlLCBkLCBWKTtcbiAgICB9IGVsc2Uge1xuICAgICAgbGV0IEggPSBuZXcgTWF0cml4KG4sIG4pO1xuICAgICAgbGV0IG9ydCA9IG5ldyBGbG9hdDY0QXJyYXkobik7XG4gICAgICBmb3IgKGogPSAwOyBqIDwgbjsgaisrKSB7XG4gICAgICAgIGZvciAoaSA9IDA7IGkgPCBuOyBpKyspIHtcbiAgICAgICAgICBILnNldChpLCBqLCB2YWx1ZS5nZXQoaSwgaikpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgICBvcnRoZXMobiwgSCwgb3J0LCBWKTtcbiAgICAgIGhxcjIobiwgZSwgZCwgViwgSCk7XG4gICAgfVxuXG4gICAgdGhpcy5uID0gbjtcbiAgICB0aGlzLmUgPSBlO1xuICAgIHRoaXMuZCA9IGQ7XG4gICAgdGhpcy5WID0gVjtcbiAgfVxuXG4gIGdldCByZWFsRWlnZW52YWx1ZXMoKSB7XG4gICAgcmV0dXJuIEFycmF5LmZyb20odGhpcy5kKTtcbiAgfVxuXG4gIGdldCBpbWFnaW5hcnlFaWdlbnZhbHVlcygpIHtcbiAgICByZXR1cm4gQXJyYXkuZnJvbSh0aGlzLmUpO1xuICB9XG5cbiAgZ2V0IGVpZ2VudmVjdG9yTWF0cml4KCkge1xuICAgIHJldHVybiB0aGlzLlY7XG4gIH1cblxuICBnZXQgZGlhZ29uYWxNYXRyaXgoKSB7XG4gICAgbGV0IG4gPSB0aGlzLm47XG4gICAgbGV0IGUgPSB0aGlzLmU7XG4gICAgbGV0IGQgPSB0aGlzLmQ7XG4gICAgbGV0IFggPSBuZXcgTWF0cml4KG4sIG4pO1xuICAgIGxldCBpLCBqO1xuICAgIGZvciAoaSA9IDA7IGkgPCBuOyBpKyspIHtcbiAgICAgIGZvciAoaiA9IDA7IGogPCBuOyBqKyspIHtcbiAgICAgICAgWC5zZXQoaSwgaiwgMCk7XG4gICAgICB9XG4gICAgICBYLnNldChpLCBpLCBkW2ldKTtcbiAgICAgIGlmIChlW2ldID4gMCkge1xuICAgICAgICBYLnNldChpLCBpICsgMSwgZVtpXSk7XG4gICAgICB9IGVsc2UgaWYgKGVbaV0gPCAwKSB7XG4gICAgICAgIFguc2V0KGksIGkgLSAxLCBlW2ldKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIFg7XG4gIH1cbn1cblxuZnVuY3Rpb24gdHJlZDIobiwgZSwgZCwgVikge1xuICBsZXQgZiwgZywgaCwgaSwgaiwgaywgaGgsIHNjYWxlO1xuXG4gIGZvciAoaiA9IDA7IGogPCBuOyBqKyspIHtcbiAgICBkW2pdID0gVi5nZXQobiAtIDEsIGopO1xuICB9XG5cbiAgZm9yIChpID0gbiAtIDE7IGkgPiAwOyBpLS0pIHtcbiAgICBzY2FsZSA9IDA7XG4gICAgaCA9IDA7XG4gICAgZm9yIChrID0gMDsgayA8IGk7IGsrKykge1xuICAgICAgc2NhbGUgPSBzY2FsZSArIE1hdGguYWJzKGRba10pO1xuICAgIH1cblxuICAgIGlmIChzY2FsZSA9PT0gMCkge1xuICAgICAgZVtpXSA9IGRbaSAtIDFdO1xuICAgICAgZm9yIChqID0gMDsgaiA8IGk7IGorKykge1xuICAgICAgICBkW2pdID0gVi5nZXQoaSAtIDEsIGopO1xuICAgICAgICBWLnNldChpLCBqLCAwKTtcbiAgICAgICAgVi5zZXQoaiwgaSwgMCk7XG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIGZvciAoayA9IDA7IGsgPCBpOyBrKyspIHtcbiAgICAgICAgZFtrXSAvPSBzY2FsZTtcbiAgICAgICAgaCArPSBkW2tdICogZFtrXTtcbiAgICAgIH1cblxuICAgICAgZiA9IGRbaSAtIDFdO1xuICAgICAgZyA9IE1hdGguc3FydChoKTtcbiAgICAgIGlmIChmID4gMCkge1xuICAgICAgICBnID0gLWc7XG4gICAgICB9XG5cbiAgICAgIGVbaV0gPSBzY2FsZSAqIGc7XG4gICAgICBoID0gaCAtIGYgKiBnO1xuICAgICAgZFtpIC0gMV0gPSBmIC0gZztcbiAgICAgIGZvciAoaiA9IDA7IGogPCBpOyBqKyspIHtcbiAgICAgICAgZVtqXSA9IDA7XG4gICAgICB9XG5cbiAgICAgIGZvciAoaiA9IDA7IGogPCBpOyBqKyspIHtcbiAgICAgICAgZiA9IGRbal07XG4gICAgICAgIFYuc2V0KGosIGksIGYpO1xuICAgICAgICBnID0gZVtqXSArIFYuZ2V0KGosIGopICogZjtcbiAgICAgICAgZm9yIChrID0gaiArIDE7IGsgPD0gaSAtIDE7IGsrKykge1xuICAgICAgICAgIGcgKz0gVi5nZXQoaywgaikgKiBkW2tdO1xuICAgICAgICAgIGVba10gKz0gVi5nZXQoaywgaikgKiBmO1xuICAgICAgICB9XG4gICAgICAgIGVbal0gPSBnO1xuICAgICAgfVxuXG4gICAgICBmID0gMDtcbiAgICAgIGZvciAoaiA9IDA7IGogPCBpOyBqKyspIHtcbiAgICAgICAgZVtqXSAvPSBoO1xuICAgICAgICBmICs9IGVbal0gKiBkW2pdO1xuICAgICAgfVxuXG4gICAgICBoaCA9IGYgLyAoaCArIGgpO1xuICAgICAgZm9yIChqID0gMDsgaiA8IGk7IGorKykge1xuICAgICAgICBlW2pdIC09IGhoICogZFtqXTtcbiAgICAgIH1cblxuICAgICAgZm9yIChqID0gMDsgaiA8IGk7IGorKykge1xuICAgICAgICBmID0gZFtqXTtcbiAgICAgICAgZyA9IGVbal07XG4gICAgICAgIGZvciAoayA9IGo7IGsgPD0gaSAtIDE7IGsrKykge1xuICAgICAgICAgIFYuc2V0KGssIGosIFYuZ2V0KGssIGopIC0gKGYgKiBlW2tdICsgZyAqIGRba10pKTtcbiAgICAgICAgfVxuICAgICAgICBkW2pdID0gVi5nZXQoaSAtIDEsIGopO1xuICAgICAgICBWLnNldChpLCBqLCAwKTtcbiAgICAgIH1cbiAgICB9XG4gICAgZFtpXSA9IGg7XG4gIH1cblxuICBmb3IgKGkgPSAwOyBpIDwgbiAtIDE7IGkrKykge1xuICAgIFYuc2V0KG4gLSAxLCBpLCBWLmdldChpLCBpKSk7XG4gICAgVi5zZXQoaSwgaSwgMSk7XG4gICAgaCA9IGRbaSArIDFdO1xuICAgIGlmIChoICE9PSAwKSB7XG4gICAgICBmb3IgKGsgPSAwOyBrIDw9IGk7IGsrKykge1xuICAgICAgICBkW2tdID0gVi5nZXQoaywgaSArIDEpIC8gaDtcbiAgICAgIH1cblxuICAgICAgZm9yIChqID0gMDsgaiA8PSBpOyBqKyspIHtcbiAgICAgICAgZyA9IDA7XG4gICAgICAgIGZvciAoayA9IDA7IGsgPD0gaTsgaysrKSB7XG4gICAgICAgICAgZyArPSBWLmdldChrLCBpICsgMSkgKiBWLmdldChrLCBqKTtcbiAgICAgICAgfVxuICAgICAgICBmb3IgKGsgPSAwOyBrIDw9IGk7IGsrKykge1xuICAgICAgICAgIFYuc2V0KGssIGosIFYuZ2V0KGssIGopIC0gZyAqIGRba10pO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuXG4gICAgZm9yIChrID0gMDsgayA8PSBpOyBrKyspIHtcbiAgICAgIFYuc2V0KGssIGkgKyAxLCAwKTtcbiAgICB9XG4gIH1cblxuICBmb3IgKGogPSAwOyBqIDwgbjsgaisrKSB7XG4gICAgZFtqXSA9IFYuZ2V0KG4gLSAxLCBqKTtcbiAgICBWLnNldChuIC0gMSwgaiwgMCk7XG4gIH1cblxuICBWLnNldChuIC0gMSwgbiAtIDEsIDEpO1xuICBlWzBdID0gMDtcbn1cblxuZnVuY3Rpb24gdHFsMihuLCBlLCBkLCBWKSB7XG4gIGxldCBnLCBoLCBpLCBqLCBrLCBsLCBtLCBwLCByLCBkbDEsIGMsIGMyLCBjMywgZWwxLCBzLCBzMiwgaXRlcjtcblxuICBmb3IgKGkgPSAxOyBpIDwgbjsgaSsrKSB7XG4gICAgZVtpIC0gMV0gPSBlW2ldO1xuICB9XG5cbiAgZVtuIC0gMV0gPSAwO1xuXG4gIGxldCBmID0gMDtcbiAgbGV0IHRzdDEgPSAwO1xuICBsZXQgZXBzID0gTnVtYmVyLkVQU0lMT047XG5cbiAgZm9yIChsID0gMDsgbCA8IG47IGwrKykge1xuICAgIHRzdDEgPSBNYXRoLm1heCh0c3QxLCBNYXRoLmFicyhkW2xdKSArIE1hdGguYWJzKGVbbF0pKTtcbiAgICBtID0gbDtcbiAgICB3aGlsZSAobSA8IG4pIHtcbiAgICAgIGlmIChNYXRoLmFicyhlW21dKSA8PSBlcHMgKiB0c3QxKSB7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuICAgICAgbSsrO1xuICAgIH1cblxuICAgIGlmIChtID4gbCkge1xuICAgICAgaXRlciA9IDA7XG4gICAgICBkbyB7XG4gICAgICAgIGl0ZXIgPSBpdGVyICsgMTtcblxuICAgICAgICBnID0gZFtsXTtcbiAgICAgICAgcCA9IChkW2wgKyAxXSAtIGcpIC8gKDIgKiBlW2xdKTtcbiAgICAgICAgciA9IGh5cG90ZW51c2UocCwgMSk7XG4gICAgICAgIGlmIChwIDwgMCkge1xuICAgICAgICAgIHIgPSAtcjtcbiAgICAgICAgfVxuXG4gICAgICAgIGRbbF0gPSBlW2xdIC8gKHAgKyByKTtcbiAgICAgICAgZFtsICsgMV0gPSBlW2xdICogKHAgKyByKTtcbiAgICAgICAgZGwxID0gZFtsICsgMV07XG4gICAgICAgIGggPSBnIC0gZFtsXTtcbiAgICAgICAgZm9yIChpID0gbCArIDI7IGkgPCBuOyBpKyspIHtcbiAgICAgICAgICBkW2ldIC09IGg7XG4gICAgICAgIH1cblxuICAgICAgICBmID0gZiArIGg7XG5cbiAgICAgICAgcCA9IGRbbV07XG4gICAgICAgIGMgPSAxO1xuICAgICAgICBjMiA9IGM7XG4gICAgICAgIGMzID0gYztcbiAgICAgICAgZWwxID0gZVtsICsgMV07XG4gICAgICAgIHMgPSAwO1xuICAgICAgICBzMiA9IDA7XG4gICAgICAgIGZvciAoaSA9IG0gLSAxOyBpID49IGw7IGktLSkge1xuICAgICAgICAgIGMzID0gYzI7XG4gICAgICAgICAgYzIgPSBjO1xuICAgICAgICAgIHMyID0gcztcbiAgICAgICAgICBnID0gYyAqIGVbaV07XG4gICAgICAgICAgaCA9IGMgKiBwO1xuICAgICAgICAgIHIgPSBoeXBvdGVudXNlKHAsIGVbaV0pO1xuICAgICAgICAgIGVbaSArIDFdID0gcyAqIHI7XG4gICAgICAgICAgcyA9IGVbaV0gLyByO1xuICAgICAgICAgIGMgPSBwIC8gcjtcbiAgICAgICAgICBwID0gYyAqIGRbaV0gLSBzICogZztcbiAgICAgICAgICBkW2kgKyAxXSA9IGggKyBzICogKGMgKiBnICsgcyAqIGRbaV0pO1xuXG4gICAgICAgICAgZm9yIChrID0gMDsgayA8IG47IGsrKykge1xuICAgICAgICAgICAgaCA9IFYuZ2V0KGssIGkgKyAxKTtcbiAgICAgICAgICAgIFYuc2V0KGssIGkgKyAxLCBzICogVi5nZXQoaywgaSkgKyBjICogaCk7XG4gICAgICAgICAgICBWLnNldChrLCBpLCBjICogVi5nZXQoaywgaSkgLSBzICogaCk7XG4gICAgICAgICAgfVxuICAgICAgICB9XG5cbiAgICAgICAgcCA9ICgtcyAqIHMyICogYzMgKiBlbDEgKiBlW2xdKSAvIGRsMTtcbiAgICAgICAgZVtsXSA9IHMgKiBwO1xuICAgICAgICBkW2xdID0gYyAqIHA7XG4gICAgICB9IHdoaWxlIChNYXRoLmFicyhlW2xdKSA+IGVwcyAqIHRzdDEpO1xuICAgIH1cbiAgICBkW2xdID0gZFtsXSArIGY7XG4gICAgZVtsXSA9IDA7XG4gIH1cblxuICBmb3IgKGkgPSAwOyBpIDwgbiAtIDE7IGkrKykge1xuICAgIGsgPSBpO1xuICAgIHAgPSBkW2ldO1xuICAgIGZvciAoaiA9IGkgKyAxOyBqIDwgbjsgaisrKSB7XG4gICAgICBpZiAoZFtqXSA8IHApIHtcbiAgICAgICAgayA9IGo7XG4gICAgICAgIHAgPSBkW2pdO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChrICE9PSBpKSB7XG4gICAgICBkW2tdID0gZFtpXTtcbiAgICAgIGRbaV0gPSBwO1xuICAgICAgZm9yIChqID0gMDsgaiA8IG47IGorKykge1xuICAgICAgICBwID0gVi5nZXQoaiwgaSk7XG4gICAgICAgIFYuc2V0KGosIGksIFYuZ2V0KGosIGspKTtcbiAgICAgICAgVi5zZXQoaiwgaywgcCk7XG4gICAgICB9XG4gICAgfVxuICB9XG59XG5cbmZ1bmN0aW9uIG9ydGhlcyhuLCBILCBvcnQsIFYpIHtcbiAgbGV0IGxvdyA9IDA7XG4gIGxldCBoaWdoID0gbiAtIDE7XG4gIGxldCBmLCBnLCBoLCBpLCBqLCBtO1xuICBsZXQgc2NhbGU7XG5cbiAgZm9yIChtID0gbG93ICsgMTsgbSA8PSBoaWdoIC0gMTsgbSsrKSB7XG4gICAgc2NhbGUgPSAwO1xuICAgIGZvciAoaSA9IG07IGkgPD0gaGlnaDsgaSsrKSB7XG4gICAgICBzY2FsZSA9IHNjYWxlICsgTWF0aC5hYnMoSC5nZXQoaSwgbSAtIDEpKTtcbiAgICB9XG5cbiAgICBpZiAoc2NhbGUgIT09IDApIHtcbiAgICAgIGggPSAwO1xuICAgICAgZm9yIChpID0gaGlnaDsgaSA+PSBtOyBpLS0pIHtcbiAgICAgICAgb3J0W2ldID0gSC5nZXQoaSwgbSAtIDEpIC8gc2NhbGU7XG4gICAgICAgIGggKz0gb3J0W2ldICogb3J0W2ldO1xuICAgICAgfVxuXG4gICAgICBnID0gTWF0aC5zcXJ0KGgpO1xuICAgICAgaWYgKG9ydFttXSA+IDApIHtcbiAgICAgICAgZyA9IC1nO1xuICAgICAgfVxuXG4gICAgICBoID0gaCAtIG9ydFttXSAqIGc7XG4gICAgICBvcnRbbV0gPSBvcnRbbV0gLSBnO1xuXG4gICAgICBmb3IgKGogPSBtOyBqIDwgbjsgaisrKSB7XG4gICAgICAgIGYgPSAwO1xuICAgICAgICBmb3IgKGkgPSBoaWdoOyBpID49IG07IGktLSkge1xuICAgICAgICAgIGYgKz0gb3J0W2ldICogSC5nZXQoaSwgaik7XG4gICAgICAgIH1cblxuICAgICAgICBmID0gZiAvIGg7XG4gICAgICAgIGZvciAoaSA9IG07IGkgPD0gaGlnaDsgaSsrKSB7XG4gICAgICAgICAgSC5zZXQoaSwgaiwgSC5nZXQoaSwgaikgLSBmICogb3J0W2ldKTtcbiAgICAgICAgfVxuICAgICAgfVxuXG4gICAgICBmb3IgKGkgPSAwOyBpIDw9IGhpZ2g7IGkrKykge1xuICAgICAgICBmID0gMDtcbiAgICAgICAgZm9yIChqID0gaGlnaDsgaiA+PSBtOyBqLS0pIHtcbiAgICAgICAgICBmICs9IG9ydFtqXSAqIEguZ2V0KGksIGopO1xuICAgICAgICB9XG5cbiAgICAgICAgZiA9IGYgLyBoO1xuICAgICAgICBmb3IgKGogPSBtOyBqIDw9IGhpZ2g7IGorKykge1xuICAgICAgICAgIEguc2V0KGksIGosIEguZ2V0KGksIGopIC0gZiAqIG9ydFtqXSk7XG4gICAgICAgIH1cbiAgICAgIH1cblxuICAgICAgb3J0W21dID0gc2NhbGUgKiBvcnRbbV07XG4gICAgICBILnNldChtLCBtIC0gMSwgc2NhbGUgKiBnKTtcbiAgICB9XG4gIH1cblxuICBmb3IgKGkgPSAwOyBpIDwgbjsgaSsrKSB7XG4gICAgZm9yIChqID0gMDsgaiA8IG47IGorKykge1xuICAgICAgVi5zZXQoaSwgaiwgaSA9PT0gaiA/IDEgOiAwKTtcbiAgICB9XG4gIH1cblxuICBmb3IgKG0gPSBoaWdoIC0gMTsgbSA+PSBsb3cgKyAxOyBtLS0pIHtcbiAgICBpZiAoSC5nZXQobSwgbSAtIDEpICE9PSAwKSB7XG4gICAgICBmb3IgKGkgPSBtICsgMTsgaSA8PSBoaWdoOyBpKyspIHtcbiAgICAgICAgb3J0W2ldID0gSC5nZXQoaSwgbSAtIDEpO1xuICAgICAgfVxuXG4gICAgICBmb3IgKGogPSBtOyBqIDw9IGhpZ2g7IGorKykge1xuICAgICAgICBnID0gMDtcbiAgICAgICAgZm9yIChpID0gbTsgaSA8PSBoaWdoOyBpKyspIHtcbiAgICAgICAgICBnICs9IG9ydFtpXSAqIFYuZ2V0KGksIGopO1xuICAgICAgICB9XG5cbiAgICAgICAgZyA9IGcgLyBvcnRbbV0gLyBILmdldChtLCBtIC0gMSk7XG4gICAgICAgIGZvciAoaSA9IG07IGkgPD0gaGlnaDsgaSsrKSB7XG4gICAgICAgICAgVi5zZXQoaSwgaiwgVi5nZXQoaSwgaikgKyBnICogb3J0W2ldKTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgfVxufVxuXG5mdW5jdGlvbiBocXIyKG5uLCBlLCBkLCBWLCBIKSB7XG4gIGxldCBuID0gbm4gLSAxO1xuICBsZXQgbG93ID0gMDtcbiAgbGV0IGhpZ2ggPSBubiAtIDE7XG4gIGxldCBlcHMgPSBOdW1iZXIuRVBTSUxPTjtcbiAgbGV0IGV4c2hpZnQgPSAwO1xuICBsZXQgbm9ybSA9IDA7XG4gIGxldCBwID0gMDtcbiAgbGV0IHEgPSAwO1xuICBsZXQgciA9IDA7XG4gIGxldCBzID0gMDtcbiAgbGV0IHogPSAwO1xuICBsZXQgaXRlciA9IDA7XG4gIGxldCBpLCBqLCBrLCBsLCBtLCB0LCB3LCB4LCB5O1xuICBsZXQgcmEsIHNhLCB2ciwgdmk7XG4gIGxldCBub3RsYXN0LCBjZGl2cmVzO1xuXG4gIGZvciAoaSA9IDA7IGkgPCBubjsgaSsrKSB7XG4gICAgaWYgKGkgPCBsb3cgfHwgaSA+IGhpZ2gpIHtcbiAgICAgIGRbaV0gPSBILmdldChpLCBpKTtcbiAgICAgIGVbaV0gPSAwO1xuICAgIH1cblxuICAgIGZvciAoaiA9IE1hdGgubWF4KGkgLSAxLCAwKTsgaiA8IG5uOyBqKyspIHtcbiAgICAgIG5vcm0gPSBub3JtICsgTWF0aC5hYnMoSC5nZXQoaSwgaikpO1xuICAgIH1cbiAgfVxuXG4gIHdoaWxlIChuID49IGxvdykge1xuICAgIGwgPSBuO1xuICAgIHdoaWxlIChsID4gbG93KSB7XG4gICAgICBzID0gTWF0aC5hYnMoSC5nZXQobCAtIDEsIGwgLSAxKSkgKyBNYXRoLmFicyhILmdldChsLCBsKSk7XG4gICAgICBpZiAocyA9PT0gMCkge1xuICAgICAgICBzID0gbm9ybTtcbiAgICAgIH1cbiAgICAgIGlmIChNYXRoLmFicyhILmdldChsLCBsIC0gMSkpIDwgZXBzICogcykge1xuICAgICAgICBicmVhaztcbiAgICAgIH1cbiAgICAgIGwtLTtcbiAgICB9XG5cbiAgICBpZiAobCA9PT0gbikge1xuICAgICAgSC5zZXQobiwgbiwgSC5nZXQobiwgbikgKyBleHNoaWZ0KTtcbiAgICAgIGRbbl0gPSBILmdldChuLCBuKTtcbiAgICAgIGVbbl0gPSAwO1xuICAgICAgbi0tO1xuICAgICAgaXRlciA9IDA7XG4gICAgfSBlbHNlIGlmIChsID09PSBuIC0gMSkge1xuICAgICAgdyA9IEguZ2V0KG4sIG4gLSAxKSAqIEguZ2V0KG4gLSAxLCBuKTtcbiAgICAgIHAgPSAoSC5nZXQobiAtIDEsIG4gLSAxKSAtIEguZ2V0KG4sIG4pKSAvIDI7XG4gICAgICBxID0gcCAqIHAgKyB3O1xuICAgICAgeiA9IE1hdGguc3FydChNYXRoLmFicyhxKSk7XG4gICAgICBILnNldChuLCBuLCBILmdldChuLCBuKSArIGV4c2hpZnQpO1xuICAgICAgSC5zZXQobiAtIDEsIG4gLSAxLCBILmdldChuIC0gMSwgbiAtIDEpICsgZXhzaGlmdCk7XG4gICAgICB4ID0gSC5nZXQobiwgbik7XG5cbiAgICAgIGlmIChxID49IDApIHtcbiAgICAgICAgeiA9IHAgPj0gMCA/IHAgKyB6IDogcCAtIHo7XG4gICAgICAgIGRbbiAtIDFdID0geCArIHo7XG4gICAgICAgIGRbbl0gPSBkW24gLSAxXTtcbiAgICAgICAgaWYgKHogIT09IDApIHtcbiAgICAgICAgICBkW25dID0geCAtIHcgLyB6O1xuICAgICAgICB9XG4gICAgICAgIGVbbiAtIDFdID0gMDtcbiAgICAgICAgZVtuXSA9IDA7XG4gICAgICAgIHggPSBILmdldChuLCBuIC0gMSk7XG4gICAgICAgIHMgPSBNYXRoLmFicyh4KSArIE1hdGguYWJzKHopO1xuICAgICAgICBwID0geCAvIHM7XG4gICAgICAgIHEgPSB6IC8gcztcbiAgICAgICAgciA9IE1hdGguc3FydChwICogcCArIHEgKiBxKTtcbiAgICAgICAgcCA9IHAgLyByO1xuICAgICAgICBxID0gcSAvIHI7XG5cbiAgICAgICAgZm9yIChqID0gbiAtIDE7IGogPCBubjsgaisrKSB7XG4gICAgICAgICAgeiA9IEguZ2V0KG4gLSAxLCBqKTtcbiAgICAgICAgICBILnNldChuIC0gMSwgaiwgcSAqIHogKyBwICogSC5nZXQobiwgaikpO1xuICAgICAgICAgIEguc2V0KG4sIGosIHEgKiBILmdldChuLCBqKSAtIHAgKiB6KTtcbiAgICAgICAgfVxuXG4gICAgICAgIGZvciAoaSA9IDA7IGkgPD0gbjsgaSsrKSB7XG4gICAgICAgICAgeiA9IEguZ2V0KGksIG4gLSAxKTtcbiAgICAgICAgICBILnNldChpLCBuIC0gMSwgcSAqIHogKyBwICogSC5nZXQoaSwgbikpO1xuICAgICAgICAgIEguc2V0KGksIG4sIHEgKiBILmdldChpLCBuKSAtIHAgKiB6KTtcbiAgICAgICAgfVxuXG4gICAgICAgIGZvciAoaSA9IGxvdzsgaSA8PSBoaWdoOyBpKyspIHtcbiAgICAgICAgICB6ID0gVi5nZXQoaSwgbiAtIDEpO1xuICAgICAgICAgIFYuc2V0KGksIG4gLSAxLCBxICogeiArIHAgKiBWLmdldChpLCBuKSk7XG4gICAgICAgICAgVi5zZXQoaSwgbiwgcSAqIFYuZ2V0KGksIG4pIC0gcCAqIHopO1xuICAgICAgICB9XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBkW24gLSAxXSA9IHggKyBwO1xuICAgICAgICBkW25dID0geCArIHA7XG4gICAgICAgIGVbbiAtIDFdID0gejtcbiAgICAgICAgZVtuXSA9IC16O1xuICAgICAgfVxuXG4gICAgICBuID0gbiAtIDI7XG4gICAgICBpdGVyID0gMDtcbiAgICB9IGVsc2Uge1xuICAgICAgeCA9IEguZ2V0KG4sIG4pO1xuICAgICAgeSA9IDA7XG4gICAgICB3ID0gMDtcbiAgICAgIGlmIChsIDwgbikge1xuICAgICAgICB5ID0gSC5nZXQobiAtIDEsIG4gLSAxKTtcbiAgICAgICAgdyA9IEguZ2V0KG4sIG4gLSAxKSAqIEguZ2V0KG4gLSAxLCBuKTtcbiAgICAgIH1cblxuICAgICAgaWYgKGl0ZXIgPT09IDEwKSB7XG4gICAgICAgIGV4c2hpZnQgKz0geDtcbiAgICAgICAgZm9yIChpID0gbG93OyBpIDw9IG47IGkrKykge1xuICAgICAgICAgIEguc2V0KGksIGksIEguZ2V0KGksIGkpIC0geCk7XG4gICAgICAgIH1cbiAgICAgICAgcyA9IE1hdGguYWJzKEguZ2V0KG4sIG4gLSAxKSkgKyBNYXRoLmFicyhILmdldChuIC0gMSwgbiAtIDIpKTtcbiAgICAgICAgeCA9IHkgPSAwLjc1ICogcztcbiAgICAgICAgdyA9IC0wLjQzNzUgKiBzICogcztcbiAgICAgIH1cblxuICAgICAgaWYgKGl0ZXIgPT09IDMwKSB7XG4gICAgICAgIHMgPSAoeSAtIHgpIC8gMjtcbiAgICAgICAgcyA9IHMgKiBzICsgdztcbiAgICAgICAgaWYgKHMgPiAwKSB7XG4gICAgICAgICAgcyA9IE1hdGguc3FydChzKTtcbiAgICAgICAgICBpZiAoeSA8IHgpIHtcbiAgICAgICAgICAgIHMgPSAtcztcbiAgICAgICAgICB9XG4gICAgICAgICAgcyA9IHggLSB3IC8gKCh5IC0geCkgLyAyICsgcyk7XG4gICAgICAgICAgZm9yIChpID0gbG93OyBpIDw9IG47IGkrKykge1xuICAgICAgICAgICAgSC5zZXQoaSwgaSwgSC5nZXQoaSwgaSkgLSBzKTtcbiAgICAgICAgICB9XG4gICAgICAgICAgZXhzaGlmdCArPSBzO1xuICAgICAgICAgIHggPSB5ID0gdyA9IDAuOTY0O1xuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIGl0ZXIgPSBpdGVyICsgMTtcblxuICAgICAgbSA9IG4gLSAyO1xuICAgICAgd2hpbGUgKG0gPj0gbCkge1xuICAgICAgICB6ID0gSC5nZXQobSwgbSk7XG4gICAgICAgIHIgPSB4IC0gejtcbiAgICAgICAgcyA9IHkgLSB6O1xuICAgICAgICBwID0gKHIgKiBzIC0gdykgLyBILmdldChtICsgMSwgbSkgKyBILmdldChtLCBtICsgMSk7XG4gICAgICAgIHEgPSBILmdldChtICsgMSwgbSArIDEpIC0geiAtIHIgLSBzO1xuICAgICAgICByID0gSC5nZXQobSArIDIsIG0gKyAxKTtcbiAgICAgICAgcyA9IE1hdGguYWJzKHApICsgTWF0aC5hYnMocSkgKyBNYXRoLmFicyhyKTtcbiAgICAgICAgcCA9IHAgLyBzO1xuICAgICAgICBxID0gcSAvIHM7XG4gICAgICAgIHIgPSByIC8gcztcbiAgICAgICAgaWYgKG0gPT09IGwpIHtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgICBpZiAoXG4gICAgICAgICAgTWF0aC5hYnMoSC5nZXQobSwgbSAtIDEpKSAqIChNYXRoLmFicyhxKSArIE1hdGguYWJzKHIpKSA8XG4gICAgICAgICAgZXBzICpcbiAgICAgICAgICAgIChNYXRoLmFicyhwKSAqXG4gICAgICAgICAgICAgIChNYXRoLmFicyhILmdldChtIC0gMSwgbSAtIDEpKSArXG4gICAgICAgICAgICAgICAgTWF0aC5hYnMoeikgK1xuICAgICAgICAgICAgICAgIE1hdGguYWJzKEguZ2V0KG0gKyAxLCBtICsgMSkpKSlcbiAgICAgICAgKSB7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIH1cbiAgICAgICAgbS0tO1xuICAgICAgfVxuXG4gICAgICBmb3IgKGkgPSBtICsgMjsgaSA8PSBuOyBpKyspIHtcbiAgICAgICAgSC5zZXQoaSwgaSAtIDIsIDApO1xuICAgICAgICBpZiAoaSA+IG0gKyAyKSB7XG4gICAgICAgICAgSC5zZXQoaSwgaSAtIDMsIDApO1xuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIGZvciAoayA9IG07IGsgPD0gbiAtIDE7IGsrKykge1xuICAgICAgICBub3RsYXN0ID0gayAhPT0gbiAtIDE7XG4gICAgICAgIGlmIChrICE9PSBtKSB7XG4gICAgICAgICAgcCA9IEguZ2V0KGssIGsgLSAxKTtcbiAgICAgICAgICBxID0gSC5nZXQoayArIDEsIGsgLSAxKTtcbiAgICAgICAgICByID0gbm90bGFzdCA/IEguZ2V0KGsgKyAyLCBrIC0gMSkgOiAwO1xuICAgICAgICAgIHggPSBNYXRoLmFicyhwKSArIE1hdGguYWJzKHEpICsgTWF0aC5hYnMocik7XG4gICAgICAgICAgaWYgKHggIT09IDApIHtcbiAgICAgICAgICAgIHAgPSBwIC8geDtcbiAgICAgICAgICAgIHEgPSBxIC8geDtcbiAgICAgICAgICAgIHIgPSByIC8geDtcbiAgICAgICAgICB9XG4gICAgICAgIH1cblxuICAgICAgICBpZiAoeCA9PT0gMCkge1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICB9XG5cbiAgICAgICAgcyA9IE1hdGguc3FydChwICogcCArIHEgKiBxICsgciAqIHIpO1xuICAgICAgICBpZiAocCA8IDApIHtcbiAgICAgICAgICBzID0gLXM7XG4gICAgICAgIH1cblxuICAgICAgICBpZiAocyAhPT0gMCkge1xuICAgICAgICAgIGlmIChrICE9PSBtKSB7XG4gICAgICAgICAgICBILnNldChrLCBrIC0gMSwgLXMgKiB4KTtcbiAgICAgICAgICB9IGVsc2UgaWYgKGwgIT09IG0pIHtcbiAgICAgICAgICAgIEguc2V0KGssIGsgLSAxLCAtSC5nZXQoaywgayAtIDEpKTtcbiAgICAgICAgICB9XG5cbiAgICAgICAgICBwID0gcCArIHM7XG4gICAgICAgICAgeCA9IHAgLyBzO1xuICAgICAgICAgIHkgPSBxIC8gcztcbiAgICAgICAgICB6ID0gciAvIHM7XG4gICAgICAgICAgcSA9IHEgLyBwO1xuICAgICAgICAgIHIgPSByIC8gcDtcblxuICAgICAgICAgIGZvciAoaiA9IGs7IGogPCBubjsgaisrKSB7XG4gICAgICAgICAgICBwID0gSC5nZXQoaywgaikgKyBxICogSC5nZXQoayArIDEsIGopO1xuICAgICAgICAgICAgaWYgKG5vdGxhc3QpIHtcbiAgICAgICAgICAgICAgcCA9IHAgKyByICogSC5nZXQoayArIDIsIGopO1xuICAgICAgICAgICAgICBILnNldChrICsgMiwgaiwgSC5nZXQoayArIDIsIGopIC0gcCAqIHopO1xuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICBILnNldChrLCBqLCBILmdldChrLCBqKSAtIHAgKiB4KTtcbiAgICAgICAgICAgIEguc2V0KGsgKyAxLCBqLCBILmdldChrICsgMSwgaikgLSBwICogeSk7XG4gICAgICAgICAgfVxuXG4gICAgICAgICAgZm9yIChpID0gMDsgaSA8PSBNYXRoLm1pbihuLCBrICsgMyk7IGkrKykge1xuICAgICAgICAgICAgcCA9IHggKiBILmdldChpLCBrKSArIHkgKiBILmdldChpLCBrICsgMSk7XG4gICAgICAgICAgICBpZiAobm90bGFzdCkge1xuICAgICAgICAgICAgICBwID0gcCArIHogKiBILmdldChpLCBrICsgMik7XG4gICAgICAgICAgICAgIEguc2V0KGksIGsgKyAyLCBILmdldChpLCBrICsgMikgLSBwICogcik7XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIEguc2V0KGksIGssIEguZ2V0KGksIGspIC0gcCk7XG4gICAgICAgICAgICBILnNldChpLCBrICsgMSwgSC5nZXQoaSwgayArIDEpIC0gcCAqIHEpO1xuICAgICAgICAgIH1cblxuICAgICAgICAgIGZvciAoaSA9IGxvdzsgaSA8PSBoaWdoOyBpKyspIHtcbiAgICAgICAgICAgIHAgPSB4ICogVi5nZXQoaSwgaykgKyB5ICogVi5nZXQoaSwgayArIDEpO1xuICAgICAgICAgICAgaWYgKG5vdGxhc3QpIHtcbiAgICAgICAgICAgICAgcCA9IHAgKyB6ICogVi5nZXQoaSwgayArIDIpO1xuICAgICAgICAgICAgICBWLnNldChpLCBrICsgMiwgVi5nZXQoaSwgayArIDIpIC0gcCAqIHIpO1xuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICBWLnNldChpLCBrLCBWLmdldChpLCBrKSAtIHApO1xuICAgICAgICAgICAgVi5zZXQoaSwgayArIDEsIFYuZ2V0KGksIGsgKyAxKSAtIHAgKiBxKTtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICBpZiAobm9ybSA9PT0gMCkge1xuICAgIHJldHVybjtcbiAgfVxuXG4gIGZvciAobiA9IG5uIC0gMTsgbiA+PSAwOyBuLS0pIHtcbiAgICBwID0gZFtuXTtcbiAgICBxID0gZVtuXTtcblxuICAgIGlmIChxID09PSAwKSB7XG4gICAgICBsID0gbjtcbiAgICAgIEguc2V0KG4sIG4sIDEpO1xuICAgICAgZm9yIChpID0gbiAtIDE7IGkgPj0gMDsgaS0tKSB7XG4gICAgICAgIHcgPSBILmdldChpLCBpKSAtIHA7XG4gICAgICAgIHIgPSAwO1xuICAgICAgICBmb3IgKGogPSBsOyBqIDw9IG47IGorKykge1xuICAgICAgICAgIHIgPSByICsgSC5nZXQoaSwgaikgKiBILmdldChqLCBuKTtcbiAgICAgICAgfVxuXG4gICAgICAgIGlmIChlW2ldIDwgMCkge1xuICAgICAgICAgIHogPSB3O1xuICAgICAgICAgIHMgPSByO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIGwgPSBpO1xuICAgICAgICAgIGlmIChlW2ldID09PSAwKSB7XG4gICAgICAgICAgICBILnNldChpLCBuLCB3ICE9PSAwID8gLXIgLyB3IDogLXIgLyAoZXBzICogbm9ybSkpO1xuICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICB4ID0gSC5nZXQoaSwgaSArIDEpO1xuICAgICAgICAgICAgeSA9IEguZ2V0KGkgKyAxLCBpKTtcbiAgICAgICAgICAgIHEgPSAoZFtpXSAtIHApICogKGRbaV0gLSBwKSArIGVbaV0gKiBlW2ldO1xuICAgICAgICAgICAgdCA9ICh4ICogcyAtIHogKiByKSAvIHE7XG4gICAgICAgICAgICBILnNldChpLCBuLCB0KTtcbiAgICAgICAgICAgIEguc2V0KFxuICAgICAgICAgICAgICBpICsgMSxcbiAgICAgICAgICAgICAgbixcbiAgICAgICAgICAgICAgTWF0aC5hYnMoeCkgPiBNYXRoLmFicyh6KSA/ICgtciAtIHcgKiB0KSAvIHggOiAoLXMgLSB5ICogdCkgLyB6LFxuICAgICAgICAgICAgKTtcbiAgICAgICAgICB9XG5cbiAgICAgICAgICB0ID0gTWF0aC5hYnMoSC5nZXQoaSwgbikpO1xuICAgICAgICAgIGlmIChlcHMgKiB0ICogdCA+IDEpIHtcbiAgICAgICAgICAgIGZvciAoaiA9IGk7IGogPD0gbjsgaisrKSB7XG4gICAgICAgICAgICAgIEguc2V0KGosIG4sIEguZ2V0KGosIG4pIC8gdCk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG4gICAgfSBlbHNlIGlmIChxIDwgMCkge1xuICAgICAgbCA9IG4gLSAxO1xuXG4gICAgICBpZiAoTWF0aC5hYnMoSC5nZXQobiwgbiAtIDEpKSA+IE1hdGguYWJzKEguZ2V0KG4gLSAxLCBuKSkpIHtcbiAgICAgICAgSC5zZXQobiAtIDEsIG4gLSAxLCBxIC8gSC5nZXQobiwgbiAtIDEpKTtcbiAgICAgICAgSC5zZXQobiAtIDEsIG4sIC0oSC5nZXQobiwgbikgLSBwKSAvIEguZ2V0KG4sIG4gLSAxKSk7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBjZGl2cmVzID0gY2RpdigwLCAtSC5nZXQobiAtIDEsIG4pLCBILmdldChuIC0gMSwgbiAtIDEpIC0gcCwgcSk7XG4gICAgICAgIEguc2V0KG4gLSAxLCBuIC0gMSwgY2RpdnJlc1swXSk7XG4gICAgICAgIEguc2V0KG4gLSAxLCBuLCBjZGl2cmVzWzFdKTtcbiAgICAgIH1cblxuICAgICAgSC5zZXQobiwgbiAtIDEsIDApO1xuICAgICAgSC5zZXQobiwgbiwgMSk7XG4gICAgICBmb3IgKGkgPSBuIC0gMjsgaSA+PSAwOyBpLS0pIHtcbiAgICAgICAgcmEgPSAwO1xuICAgICAgICBzYSA9IDA7XG4gICAgICAgIGZvciAoaiA9IGw7IGogPD0gbjsgaisrKSB7XG4gICAgICAgICAgcmEgPSByYSArIEguZ2V0KGksIGopICogSC5nZXQoaiwgbiAtIDEpO1xuICAgICAgICAgIHNhID0gc2EgKyBILmdldChpLCBqKSAqIEguZ2V0KGosIG4pO1xuICAgICAgICB9XG5cbiAgICAgICAgdyA9IEguZ2V0KGksIGkpIC0gcDtcblxuICAgICAgICBpZiAoZVtpXSA8IDApIHtcbiAgICAgICAgICB6ID0gdztcbiAgICAgICAgICByID0gcmE7XG4gICAgICAgICAgcyA9IHNhO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIGwgPSBpO1xuICAgICAgICAgIGlmIChlW2ldID09PSAwKSB7XG4gICAgICAgICAgICBjZGl2cmVzID0gY2RpdigtcmEsIC1zYSwgdywgcSk7XG4gICAgICAgICAgICBILnNldChpLCBuIC0gMSwgY2RpdnJlc1swXSk7XG4gICAgICAgICAgICBILnNldChpLCBuLCBjZGl2cmVzWzFdKTtcbiAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgeCA9IEguZ2V0KGksIGkgKyAxKTtcbiAgICAgICAgICAgIHkgPSBILmdldChpICsgMSwgaSk7XG4gICAgICAgICAgICB2ciA9IChkW2ldIC0gcCkgKiAoZFtpXSAtIHApICsgZVtpXSAqIGVbaV0gLSBxICogcTtcbiAgICAgICAgICAgIHZpID0gKGRbaV0gLSBwKSAqIDIgKiBxO1xuICAgICAgICAgICAgaWYgKHZyID09PSAwICYmIHZpID09PSAwKSB7XG4gICAgICAgICAgICAgIHZyID1cbiAgICAgICAgICAgICAgICBlcHMgKlxuICAgICAgICAgICAgICAgIG5vcm0gKlxuICAgICAgICAgICAgICAgIChNYXRoLmFicyh3KSArXG4gICAgICAgICAgICAgICAgICBNYXRoLmFicyhxKSArXG4gICAgICAgICAgICAgICAgICBNYXRoLmFicyh4KSArXG4gICAgICAgICAgICAgICAgICBNYXRoLmFicyh5KSArXG4gICAgICAgICAgICAgICAgICBNYXRoLmFicyh6KSk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBjZGl2cmVzID0gY2RpdihcbiAgICAgICAgICAgICAgeCAqIHIgLSB6ICogcmEgKyBxICogc2EsXG4gICAgICAgICAgICAgIHggKiBzIC0geiAqIHNhIC0gcSAqIHJhLFxuICAgICAgICAgICAgICB2cixcbiAgICAgICAgICAgICAgdmksXG4gICAgICAgICAgICApO1xuICAgICAgICAgICAgSC5zZXQoaSwgbiAtIDEsIGNkaXZyZXNbMF0pO1xuICAgICAgICAgICAgSC5zZXQoaSwgbiwgY2RpdnJlc1sxXSk7XG4gICAgICAgICAgICBpZiAoTWF0aC5hYnMoeCkgPiBNYXRoLmFicyh6KSArIE1hdGguYWJzKHEpKSB7XG4gICAgICAgICAgICAgIEguc2V0KFxuICAgICAgICAgICAgICAgIGkgKyAxLFxuICAgICAgICAgICAgICAgIG4gLSAxLFxuICAgICAgICAgICAgICAgICgtcmEgLSB3ICogSC5nZXQoaSwgbiAtIDEpICsgcSAqIEguZ2V0KGksIG4pKSAvIHgsXG4gICAgICAgICAgICAgICk7XG4gICAgICAgICAgICAgIEguc2V0KFxuICAgICAgICAgICAgICAgIGkgKyAxLFxuICAgICAgICAgICAgICAgIG4sXG4gICAgICAgICAgICAgICAgKC1zYSAtIHcgKiBILmdldChpLCBuKSAtIHEgKiBILmdldChpLCBuIC0gMSkpIC8geCxcbiAgICAgICAgICAgICAgKTtcbiAgICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICAgIGNkaXZyZXMgPSBjZGl2KFxuICAgICAgICAgICAgICAgIC1yIC0geSAqIEguZ2V0KGksIG4gLSAxKSxcbiAgICAgICAgICAgICAgICAtcyAtIHkgKiBILmdldChpLCBuKSxcbiAgICAgICAgICAgICAgICB6LFxuICAgICAgICAgICAgICAgIHEsXG4gICAgICAgICAgICAgICk7XG4gICAgICAgICAgICAgIEguc2V0KGkgKyAxLCBuIC0gMSwgY2RpdnJlc1swXSk7XG4gICAgICAgICAgICAgIEguc2V0KGkgKyAxLCBuLCBjZGl2cmVzWzFdKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG5cbiAgICAgICAgICB0ID0gTWF0aC5tYXgoTWF0aC5hYnMoSC5nZXQoaSwgbiAtIDEpKSwgTWF0aC5hYnMoSC5nZXQoaSwgbikpKTtcbiAgICAgICAgICBpZiAoZXBzICogdCAqIHQgPiAxKSB7XG4gICAgICAgICAgICBmb3IgKGogPSBpOyBqIDw9IG47IGorKykge1xuICAgICAgICAgICAgICBILnNldChqLCBuIC0gMSwgSC5nZXQoaiwgbiAtIDEpIC8gdCk7XG4gICAgICAgICAgICAgIEguc2V0KGosIG4sIEguZ2V0KGosIG4pIC8gdCk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgZm9yIChpID0gMDsgaSA8IG5uOyBpKyspIHtcbiAgICBpZiAoaSA8IGxvdyB8fCBpID4gaGlnaCkge1xuICAgICAgZm9yIChqID0gaTsgaiA8IG5uOyBqKyspIHtcbiAgICAgICAgVi5zZXQoaSwgaiwgSC5nZXQoaSwgaikpO1xuICAgICAgfVxuICAgIH1cbiAgfVxuXG4gIGZvciAoaiA9IG5uIC0gMTsgaiA+PSBsb3c7IGotLSkge1xuICAgIGZvciAoaSA9IGxvdzsgaSA8PSBoaWdoOyBpKyspIHtcbiAgICAgIHogPSAwO1xuICAgICAgZm9yIChrID0gbG93OyBrIDw9IE1hdGgubWluKGosIGhpZ2gpOyBrKyspIHtcbiAgICAgICAgeiA9IHogKyBWLmdldChpLCBrKSAqIEguZ2V0KGssIGopO1xuICAgICAgfVxuICAgICAgVi5zZXQoaSwgaiwgeik7XG4gICAgfVxuICB9XG59XG5cbmZ1bmN0aW9uIGNkaXYoeHIsIHhpLCB5ciwgeWkpIHtcbiAgbGV0IHIsIGQ7XG4gIGlmIChNYXRoLmFicyh5cikgPiBNYXRoLmFicyh5aSkpIHtcbiAgICByID0geWkgLyB5cjtcbiAgICBkID0geXIgKyByICogeWk7XG4gICAgcmV0dXJuIFsoeHIgKyByICogeGkpIC8gZCwgKHhpIC0gciAqIHhyKSAvIGRdO1xuICB9IGVsc2Uge1xuICAgIHIgPSB5ciAvIHlpO1xuICAgIGQgPSB5aSArIHIgKiB5cjtcbiAgICByZXR1cm4gWyhyICogeHIgKyB4aSkgLyBkLCAociAqIHhpIC0geHIpIC8gZF07XG4gIH1cbn1cbiIsImltcG9ydCBNYXRyaXggZnJvbSAnLi4vbWF0cml4JztcbmltcG9ydCBXcmFwcGVyTWF0cml4MkQgZnJvbSAnLi4vd3JhcC9XcmFwcGVyTWF0cml4MkQnO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBMdURlY29tcG9zaXRpb24ge1xuICBjb25zdHJ1Y3RvcihtYXRyaXgpIHtcbiAgICBtYXRyaXggPSBXcmFwcGVyTWF0cml4MkQuY2hlY2tNYXRyaXgobWF0cml4KTtcblxuICAgIGxldCBsdSA9IG1hdHJpeC5jbG9uZSgpO1xuICAgIGxldCByb3dzID0gbHUucm93cztcbiAgICBsZXQgY29sdW1ucyA9IGx1LmNvbHVtbnM7XG4gICAgbGV0IHBpdm90VmVjdG9yID0gbmV3IEZsb2F0NjRBcnJheShyb3dzKTtcbiAgICBsZXQgcGl2b3RTaWduID0gMTtcbiAgICBsZXQgaSwgaiwgaywgcCwgcywgdCwgdjtcbiAgICBsZXQgTFVjb2xqLCBrbWF4O1xuXG4gICAgZm9yIChpID0gMDsgaSA8IHJvd3M7IGkrKykge1xuICAgICAgcGl2b3RWZWN0b3JbaV0gPSBpO1xuICAgIH1cblxuICAgIExVY29saiA9IG5ldyBGbG9hdDY0QXJyYXkocm93cyk7XG5cbiAgICBmb3IgKGogPSAwOyBqIDwgY29sdW1uczsgaisrKSB7XG4gICAgICBmb3IgKGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICAgIExVY29saltpXSA9IGx1LmdldChpLCBqKTtcbiAgICAgIH1cblxuICAgICAgZm9yIChpID0gMDsgaSA8IHJvd3M7IGkrKykge1xuICAgICAgICBrbWF4ID0gTWF0aC5taW4oaSwgaik7XG4gICAgICAgIHMgPSAwO1xuICAgICAgICBmb3IgKGsgPSAwOyBrIDwga21heDsgaysrKSB7XG4gICAgICAgICAgcyArPSBsdS5nZXQoaSwgaykgKiBMVWNvbGpba107XG4gICAgICAgIH1cbiAgICAgICAgTFVjb2xqW2ldIC09IHM7XG4gICAgICAgIGx1LnNldChpLCBqLCBMVWNvbGpbaV0pO1xuICAgICAgfVxuXG4gICAgICBwID0gajtcbiAgICAgIGZvciAoaSA9IGogKyAxOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICAgIGlmIChNYXRoLmFicyhMVWNvbGpbaV0pID4gTWF0aC5hYnMoTFVjb2xqW3BdKSkge1xuICAgICAgICAgIHAgPSBpO1xuICAgICAgICB9XG4gICAgICB9XG5cbiAgICAgIGlmIChwICE9PSBqKSB7XG4gICAgICAgIGZvciAoayA9IDA7IGsgPCBjb2x1bW5zOyBrKyspIHtcbiAgICAgICAgICB0ID0gbHUuZ2V0KHAsIGspO1xuICAgICAgICAgIGx1LnNldChwLCBrLCBsdS5nZXQoaiwgaykpO1xuICAgICAgICAgIGx1LnNldChqLCBrLCB0KTtcbiAgICAgICAgfVxuXG4gICAgICAgIHYgPSBwaXZvdFZlY3RvcltwXTtcbiAgICAgICAgcGl2b3RWZWN0b3JbcF0gPSBwaXZvdFZlY3RvcltqXTtcbiAgICAgICAgcGl2b3RWZWN0b3Jbal0gPSB2O1xuXG4gICAgICAgIHBpdm90U2lnbiA9IC1waXZvdFNpZ247XG4gICAgICB9XG5cbiAgICAgIGlmIChqIDwgcm93cyAmJiBsdS5nZXQoaiwgaikgIT09IDApIHtcbiAgICAgICAgZm9yIChpID0gaiArIDE7IGkgPCByb3dzOyBpKyspIHtcbiAgICAgICAgICBsdS5zZXQoaSwgaiwgbHUuZ2V0KGksIGopIC8gbHUuZ2V0KGosIGopKTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cblxuICAgIHRoaXMuTFUgPSBsdTtcbiAgICB0aGlzLnBpdm90VmVjdG9yID0gcGl2b3RWZWN0b3I7XG4gICAgdGhpcy5waXZvdFNpZ24gPSBwaXZvdFNpZ247XG4gIH1cblxuICBpc1Npbmd1bGFyKCkge1xuICAgIGxldCBkYXRhID0gdGhpcy5MVTtcbiAgICBsZXQgY29sID0gZGF0YS5jb2x1bW5zO1xuICAgIGZvciAobGV0IGogPSAwOyBqIDwgY29sOyBqKyspIHtcbiAgICAgIGlmIChkYXRhLmdldChqLCBqKSA9PT0gMCkge1xuICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgc29sdmUodmFsdWUpIHtcbiAgICB2YWx1ZSA9IE1hdHJpeC5jaGVja01hdHJpeCh2YWx1ZSk7XG5cbiAgICBsZXQgbHUgPSB0aGlzLkxVO1xuICAgIGxldCByb3dzID0gbHUucm93cztcblxuICAgIGlmIChyb3dzICE9PSB2YWx1ZS5yb3dzKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoJ0ludmFsaWQgbWF0cml4IGRpbWVuc2lvbnMnKTtcbiAgICB9XG4gICAgaWYgKHRoaXMuaXNTaW5ndWxhcigpKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoJ0xVIG1hdHJpeCBpcyBzaW5ndWxhcicpO1xuICAgIH1cblxuICAgIGxldCBjb3VudCA9IHZhbHVlLmNvbHVtbnM7XG4gICAgbGV0IFggPSB2YWx1ZS5zdWJNYXRyaXhSb3codGhpcy5waXZvdFZlY3RvciwgMCwgY291bnQgLSAxKTtcbiAgICBsZXQgY29sdW1ucyA9IGx1LmNvbHVtbnM7XG4gICAgbGV0IGksIGosIGs7XG5cbiAgICBmb3IgKGsgPSAwOyBrIDwgY29sdW1uczsgaysrKSB7XG4gICAgICBmb3IgKGkgPSBrICsgMTsgaSA8IGNvbHVtbnM7IGkrKykge1xuICAgICAgICBmb3IgKGogPSAwOyBqIDwgY291bnQ7IGorKykge1xuICAgICAgICAgIFguc2V0KGksIGosIFguZ2V0KGksIGopIC0gWC5nZXQoaywgaikgKiBsdS5nZXQoaSwgaykpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICAgIGZvciAoayA9IGNvbHVtbnMgLSAxOyBrID49IDA7IGstLSkge1xuICAgICAgZm9yIChqID0gMDsgaiA8IGNvdW50OyBqKyspIHtcbiAgICAgICAgWC5zZXQoaywgaiwgWC5nZXQoaywgaikgLyBsdS5nZXQoaywgaykpO1xuICAgICAgfVxuICAgICAgZm9yIChpID0gMDsgaSA8IGs7IGkrKykge1xuICAgICAgICBmb3IgKGogPSAwOyBqIDwgY291bnQ7IGorKykge1xuICAgICAgICAgIFguc2V0KGksIGosIFguZ2V0KGksIGopIC0gWC5nZXQoaywgaikgKiBsdS5nZXQoaSwgaykpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBYO1xuICB9XG5cbiAgZ2V0IGRldGVybWluYW50KCkge1xuICAgIGxldCBkYXRhID0gdGhpcy5MVTtcbiAgICBpZiAoIWRhdGEuaXNTcXVhcmUoKSkge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKCdNYXRyaXggbXVzdCBiZSBzcXVhcmUnKTtcbiAgICB9XG4gICAgbGV0IGRldGVybWluYW50ID0gdGhpcy5waXZvdFNpZ247XG4gICAgbGV0IGNvbCA9IGRhdGEuY29sdW1ucztcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IGNvbDsgaisrKSB7XG4gICAgICBkZXRlcm1pbmFudCAqPSBkYXRhLmdldChqLCBqKTtcbiAgICB9XG4gICAgcmV0dXJuIGRldGVybWluYW50O1xuICB9XG5cbiAgZ2V0IGxvd2VyVHJpYW5ndWxhck1hdHJpeCgpIHtcbiAgICBsZXQgZGF0YSA9IHRoaXMuTFU7XG4gICAgbGV0IHJvd3MgPSBkYXRhLnJvd3M7XG4gICAgbGV0IGNvbHVtbnMgPSBkYXRhLmNvbHVtbnM7XG4gICAgbGV0IFggPSBuZXcgTWF0cml4KHJvd3MsIGNvbHVtbnMpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IGNvbHVtbnM7IGorKykge1xuICAgICAgICBpZiAoaSA+IGopIHtcbiAgICAgICAgICBYLnNldChpLCBqLCBkYXRhLmdldChpLCBqKSk7XG4gICAgICAgIH0gZWxzZSBpZiAoaSA9PT0gaikge1xuICAgICAgICAgIFguc2V0KGksIGosIDEpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIFguc2V0KGksIGosIDApO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBYO1xuICB9XG5cbiAgZ2V0IHVwcGVyVHJpYW5ndWxhck1hdHJpeCgpIHtcbiAgICBsZXQgZGF0YSA9IHRoaXMuTFU7XG4gICAgbGV0IHJvd3MgPSBkYXRhLnJvd3M7XG4gICAgbGV0IGNvbHVtbnMgPSBkYXRhLmNvbHVtbnM7XG4gICAgbGV0IFggPSBuZXcgTWF0cml4KHJvd3MsIGNvbHVtbnMpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IGNvbHVtbnM7IGorKykge1xuICAgICAgICBpZiAoaSA8PSBqKSB7XG4gICAgICAgICAgWC5zZXQoaSwgaiwgZGF0YS5nZXQoaSwgaikpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIFguc2V0KGksIGosIDApO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBYO1xuICB9XG5cbiAgZ2V0IHBpdm90UGVybXV0YXRpb25WZWN0b3IoKSB7XG4gICAgcmV0dXJuIEFycmF5LmZyb20odGhpcy5waXZvdFZlY3Rvcik7XG4gIH1cbn1cbiIsImltcG9ydCB7IGlzQW55QXJyYXkgfSBmcm9tICdpcy1hbnktYXJyYXknO1xuXG5pbXBvcnQgTWF0cml4IGZyb20gJy4uL21hdHJpeCc7XG5pbXBvcnQgV3JhcHBlck1hdHJpeDJEIGZyb20gJy4uL3dyYXAvV3JhcHBlck1hdHJpeDJEJztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgbmlwYWxzIHtcbiAgY29uc3RydWN0b3IoWCwgb3B0aW9ucyA9IHt9KSB7XG4gICAgWCA9IFdyYXBwZXJNYXRyaXgyRC5jaGVja01hdHJpeChYKTtcbiAgICBsZXQgeyBZIH0gPSBvcHRpb25zO1xuICAgIGNvbnN0IHtcbiAgICAgIHNjYWxlU2NvcmVzID0gZmFsc2UsXG4gICAgICBtYXhJdGVyYXRpb25zID0gMTAwMCxcbiAgICAgIHRlcm1pbmF0aW9uQ3JpdGVyaWEgPSAxZS0xMCxcbiAgICB9ID0gb3B0aW9ucztcblxuICAgIGxldCB1O1xuICAgIGlmIChZKSB7XG4gICAgICBpZiAoaXNBbnlBcnJheShZKSAmJiB0eXBlb2YgWVswXSA9PT0gJ251bWJlcicpIHtcbiAgICAgICAgWSA9IE1hdHJpeC5jb2x1bW5WZWN0b3IoWSk7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBZID0gV3JhcHBlck1hdHJpeDJELmNoZWNrTWF0cml4KFkpO1xuICAgICAgfVxuICAgICAgaWYgKFkucm93cyAhPT0gWC5yb3dzKSB7XG4gICAgICAgIHRocm93IG5ldyBFcnJvcignWSBzaG91bGQgaGF2ZSB0aGUgc2FtZSBudW1iZXIgb2Ygcm93cyBhcyBYJyk7XG4gICAgICB9XG4gICAgICB1ID0gWS5nZXRDb2x1bW5WZWN0b3IoMCk7XG4gICAgfSBlbHNlIHtcbiAgICAgIHUgPSBYLmdldENvbHVtblZlY3RvcigwKTtcbiAgICB9XG5cbiAgICBsZXQgZGlmZiA9IDE7XG4gICAgbGV0IHQsIHEsIHcsIHRPbGQ7XG5cbiAgICBmb3IgKFxuICAgICAgbGV0IGNvdW50ZXIgPSAwO1xuICAgICAgY291bnRlciA8IG1heEl0ZXJhdGlvbnMgJiYgZGlmZiA+IHRlcm1pbmF0aW9uQ3JpdGVyaWE7XG4gICAgICBjb3VudGVyKytcbiAgICApIHtcbiAgICAgIHcgPSBYLnRyYW5zcG9zZSgpLm1tdWwodSkuZGl2KHUudHJhbnNwb3NlKCkubW11bCh1KS5nZXQoMCwgMCkpO1xuICAgICAgdyA9IHcuZGl2KHcubm9ybSgpKTtcblxuICAgICAgdCA9IFgubW11bCh3KS5kaXYody50cmFuc3Bvc2UoKS5tbXVsKHcpLmdldCgwLCAwKSk7XG5cbiAgICAgIGlmIChjb3VudGVyID4gMCkge1xuICAgICAgICBkaWZmID0gdC5jbG9uZSgpLnN1Yih0T2xkKS5wb3coMikuc3VtKCk7XG4gICAgICB9XG4gICAgICB0T2xkID0gdC5jbG9uZSgpO1xuXG4gICAgICBpZiAoWSkge1xuICAgICAgICBxID0gWS50cmFuc3Bvc2UoKS5tbXVsKHQpLmRpdih0LnRyYW5zcG9zZSgpLm1tdWwodCkuZ2V0KDAsIDApKTtcbiAgICAgICAgcSA9IHEuZGl2KHEubm9ybSgpKTtcblxuICAgICAgICB1ID0gWS5tbXVsKHEpLmRpdihxLnRyYW5zcG9zZSgpLm1tdWwocSkuZ2V0KDAsIDApKTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHUgPSB0O1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChZKSB7XG4gICAgICBsZXQgcCA9IFgudHJhbnNwb3NlKCkubW11bCh0KS5kaXYodC50cmFuc3Bvc2UoKS5tbXVsKHQpLmdldCgwLCAwKSk7XG4gICAgICBwID0gcC5kaXYocC5ub3JtKCkpO1xuICAgICAgbGV0IHhSZXNpZHVhbCA9IFguY2xvbmUoKS5zdWIodC5jbG9uZSgpLm1tdWwocC50cmFuc3Bvc2UoKSkpO1xuICAgICAgbGV0IHJlc2lkdWFsID0gdS50cmFuc3Bvc2UoKS5tbXVsKHQpLmRpdih0LnRyYW5zcG9zZSgpLm1tdWwodCkuZ2V0KDAsIDApKTtcbiAgICAgIGxldCB5UmVzaWR1YWwgPSBZLmNsb25lKCkuc3ViKFxuICAgICAgICB0LmNsb25lKCkubXVsUyhyZXNpZHVhbC5nZXQoMCwgMCkpLm1tdWwocS50cmFuc3Bvc2UoKSksXG4gICAgICApO1xuXG4gICAgICB0aGlzLnQgPSB0O1xuICAgICAgdGhpcy5wID0gcC50cmFuc3Bvc2UoKTtcbiAgICAgIHRoaXMudyA9IHcudHJhbnNwb3NlKCk7XG4gICAgICB0aGlzLnEgPSBxO1xuICAgICAgdGhpcy51ID0gdTtcbiAgICAgIHRoaXMucyA9IHQudHJhbnNwb3NlKCkubW11bCh0KTtcbiAgICAgIHRoaXMueFJlc2lkdWFsID0geFJlc2lkdWFsO1xuICAgICAgdGhpcy55UmVzaWR1YWwgPSB5UmVzaWR1YWw7XG4gICAgICB0aGlzLmJldGFzID0gcmVzaWR1YWw7XG4gICAgfSBlbHNlIHtcbiAgICAgIHRoaXMudyA9IHcudHJhbnNwb3NlKCk7XG4gICAgICB0aGlzLnMgPSB0LnRyYW5zcG9zZSgpLm1tdWwodCkuc3FydCgpO1xuICAgICAgaWYgKHNjYWxlU2NvcmVzKSB7XG4gICAgICAgIHRoaXMudCA9IHQuY2xvbmUoKS5kaXYodGhpcy5zLmdldCgwLCAwKSk7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICB0aGlzLnQgPSB0O1xuICAgICAgfVxuICAgICAgdGhpcy54UmVzaWR1YWwgPSBYLnN1Yih0Lm1tdWwody50cmFuc3Bvc2UoKSkpO1xuICAgIH1cbiAgfVxufVxuIiwiaW1wb3J0IE1hdHJpeCBmcm9tICcuLi9tYXRyaXgnO1xuaW1wb3J0IFdyYXBwZXJNYXRyaXgyRCBmcm9tICcuLi93cmFwL1dyYXBwZXJNYXRyaXgyRCc7XG5cbmltcG9ydCB7IGh5cG90ZW51c2UgfSBmcm9tICcuL3V0aWwnO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBRckRlY29tcG9zaXRpb24ge1xuICBjb25zdHJ1Y3Rvcih2YWx1ZSkge1xuICAgIHZhbHVlID0gV3JhcHBlck1hdHJpeDJELmNoZWNrTWF0cml4KHZhbHVlKTtcblxuICAgIGxldCBxciA9IHZhbHVlLmNsb25lKCk7XG4gICAgbGV0IG0gPSB2YWx1ZS5yb3dzO1xuICAgIGxldCBuID0gdmFsdWUuY29sdW1ucztcbiAgICBsZXQgcmRpYWcgPSBuZXcgRmxvYXQ2NEFycmF5KG4pO1xuICAgIGxldCBpLCBqLCBrLCBzO1xuXG4gICAgZm9yIChrID0gMDsgayA8IG47IGsrKykge1xuICAgICAgbGV0IG5ybSA9IDA7XG4gICAgICBmb3IgKGkgPSBrOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgIG5ybSA9IGh5cG90ZW51c2UobnJtLCBxci5nZXQoaSwgaykpO1xuICAgICAgfVxuICAgICAgaWYgKG5ybSAhPT0gMCkge1xuICAgICAgICBpZiAocXIuZ2V0KGssIGspIDwgMCkge1xuICAgICAgICAgIG5ybSA9IC1ucm07XG4gICAgICAgIH1cbiAgICAgICAgZm9yIChpID0gazsgaSA8IG07IGkrKykge1xuICAgICAgICAgIHFyLnNldChpLCBrLCBxci5nZXQoaSwgaykgLyBucm0pO1xuICAgICAgICB9XG4gICAgICAgIHFyLnNldChrLCBrLCBxci5nZXQoaywgaykgKyAxKTtcbiAgICAgICAgZm9yIChqID0gayArIDE7IGogPCBuOyBqKyspIHtcbiAgICAgICAgICBzID0gMDtcbiAgICAgICAgICBmb3IgKGkgPSBrOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICBzICs9IHFyLmdldChpLCBrKSAqIHFyLmdldChpLCBqKTtcbiAgICAgICAgICB9XG4gICAgICAgICAgcyA9IC1zIC8gcXIuZ2V0KGssIGspO1xuICAgICAgICAgIGZvciAoaSA9IGs7IGkgPCBtOyBpKyspIHtcbiAgICAgICAgICAgIHFyLnNldChpLCBqLCBxci5nZXQoaSwgaikgKyBzICogcXIuZ2V0KGksIGspKTtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIHJkaWFnW2tdID0gLW5ybTtcbiAgICB9XG5cbiAgICB0aGlzLlFSID0gcXI7XG4gICAgdGhpcy5SZGlhZyA9IHJkaWFnO1xuICB9XG5cbiAgc29sdmUodmFsdWUpIHtcbiAgICB2YWx1ZSA9IE1hdHJpeC5jaGVja01hdHJpeCh2YWx1ZSk7XG5cbiAgICBsZXQgcXIgPSB0aGlzLlFSO1xuICAgIGxldCBtID0gcXIucm93cztcblxuICAgIGlmICh2YWx1ZS5yb3dzICE9PSBtKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoJ01hdHJpeCByb3cgZGltZW5zaW9ucyBtdXN0IGFncmVlJyk7XG4gICAgfVxuICAgIGlmICghdGhpcy5pc0Z1bGxSYW5rKCkpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcignTWF0cml4IGlzIHJhbmsgZGVmaWNpZW50Jyk7XG4gICAgfVxuXG4gICAgbGV0IGNvdW50ID0gdmFsdWUuY29sdW1ucztcbiAgICBsZXQgWCA9IHZhbHVlLmNsb25lKCk7XG4gICAgbGV0IG4gPSBxci5jb2x1bW5zO1xuICAgIGxldCBpLCBqLCBrLCBzO1xuXG4gICAgZm9yIChrID0gMDsgayA8IG47IGsrKykge1xuICAgICAgZm9yIChqID0gMDsgaiA8IGNvdW50OyBqKyspIHtcbiAgICAgICAgcyA9IDA7XG4gICAgICAgIGZvciAoaSA9IGs7IGkgPCBtOyBpKyspIHtcbiAgICAgICAgICBzICs9IHFyLmdldChpLCBrKSAqIFguZ2V0KGksIGopO1xuICAgICAgICB9XG4gICAgICAgIHMgPSAtcyAvIHFyLmdldChrLCBrKTtcbiAgICAgICAgZm9yIChpID0gazsgaSA8IG07IGkrKykge1xuICAgICAgICAgIFguc2V0KGksIGosIFguZ2V0KGksIGopICsgcyAqIHFyLmdldChpLCBrKSk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG4gICAgZm9yIChrID0gbiAtIDE7IGsgPj0gMDsgay0tKSB7XG4gICAgICBmb3IgKGogPSAwOyBqIDwgY291bnQ7IGorKykge1xuICAgICAgICBYLnNldChrLCBqLCBYLmdldChrLCBqKSAvIHRoaXMuUmRpYWdba10pO1xuICAgICAgfVxuICAgICAgZm9yIChpID0gMDsgaSA8IGs7IGkrKykge1xuICAgICAgICBmb3IgKGogPSAwOyBqIDwgY291bnQ7IGorKykge1xuICAgICAgICAgIFguc2V0KGksIGosIFguZ2V0KGksIGopIC0gWC5nZXQoaywgaikgKiBxci5nZXQoaSwgaykpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuXG4gICAgcmV0dXJuIFguc3ViTWF0cml4KDAsIG4gLSAxLCAwLCBjb3VudCAtIDEpO1xuICB9XG5cbiAgaXNGdWxsUmFuaygpIHtcbiAgICBsZXQgY29sdW1ucyA9IHRoaXMuUVIuY29sdW1ucztcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGNvbHVtbnM7IGkrKykge1xuICAgICAgaWYgKHRoaXMuUmRpYWdbaV0gPT09IDApIHtcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuXG4gIGdldCB1cHBlclRyaWFuZ3VsYXJNYXRyaXgoKSB7XG4gICAgbGV0IHFyID0gdGhpcy5RUjtcbiAgICBsZXQgbiA9IHFyLmNvbHVtbnM7XG4gICAgbGV0IFggPSBuZXcgTWF0cml4KG4sIG4pO1xuICAgIGxldCBpLCBqO1xuICAgIGZvciAoaSA9IDA7IGkgPCBuOyBpKyspIHtcbiAgICAgIGZvciAoaiA9IDA7IGogPCBuOyBqKyspIHtcbiAgICAgICAgaWYgKGkgPCBqKSB7XG4gICAgICAgICAgWC5zZXQoaSwgaiwgcXIuZ2V0KGksIGopKTtcbiAgICAgICAgfSBlbHNlIGlmIChpID09PSBqKSB7XG4gICAgICAgICAgWC5zZXQoaSwgaiwgdGhpcy5SZGlhZ1tpXSk7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgWC5zZXQoaSwgaiwgMCk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIFg7XG4gIH1cblxuICBnZXQgb3J0aG9nb25hbE1hdHJpeCgpIHtcbiAgICBsZXQgcXIgPSB0aGlzLlFSO1xuICAgIGxldCByb3dzID0gcXIucm93cztcbiAgICBsZXQgY29sdW1ucyA9IHFyLmNvbHVtbnM7XG4gICAgbGV0IFggPSBuZXcgTWF0cml4KHJvd3MsIGNvbHVtbnMpO1xuICAgIGxldCBpLCBqLCBrLCBzO1xuXG4gICAgZm9yIChrID0gY29sdW1ucyAtIDE7IGsgPj0gMDsgay0tKSB7XG4gICAgICBmb3IgKGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICAgIFguc2V0KGksIGssIDApO1xuICAgICAgfVxuICAgICAgWC5zZXQoaywgaywgMSk7XG4gICAgICBmb3IgKGogPSBrOyBqIDwgY29sdW1uczsgaisrKSB7XG4gICAgICAgIGlmIChxci5nZXQoaywgaykgIT09IDApIHtcbiAgICAgICAgICBzID0gMDtcbiAgICAgICAgICBmb3IgKGkgPSBrOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICAgICAgICBzICs9IHFyLmdldChpLCBrKSAqIFguZ2V0KGksIGopO1xuICAgICAgICAgIH1cblxuICAgICAgICAgIHMgPSAtcyAvIHFyLmdldChrLCBrKTtcblxuICAgICAgICAgIGZvciAoaSA9IGs7IGkgPCByb3dzOyBpKyspIHtcbiAgICAgICAgICAgIFguc2V0KGksIGosIFguZ2V0KGksIGopICsgcyAqIHFyLmdldChpLCBrKSk7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBYO1xuICB9XG59XG4iLCJpbXBvcnQgTWF0cml4IGZyb20gJy4uL21hdHJpeCc7XG5pbXBvcnQgV3JhcHBlck1hdHJpeDJEIGZyb20gJy4uL3dyYXAvV3JhcHBlck1hdHJpeDJEJztcblxuaW1wb3J0IHsgaHlwb3RlbnVzZSB9IGZyb20gJy4vdXRpbCc7XG5cbmV4cG9ydCBkZWZhdWx0IGNsYXNzIFNpbmd1bGFyVmFsdWVEZWNvbXBvc2l0aW9uIHtcbiAgY29uc3RydWN0b3IodmFsdWUsIG9wdGlvbnMgPSB7fSkge1xuICAgIHZhbHVlID0gV3JhcHBlck1hdHJpeDJELmNoZWNrTWF0cml4KHZhbHVlKTtcblxuICAgIGlmICh2YWx1ZS5pc0VtcHR5KCkpIHtcbiAgICAgIHRocm93IG5ldyBFcnJvcignTWF0cml4IG11c3QgYmUgbm9uLWVtcHR5Jyk7XG4gICAgfVxuXG4gICAgbGV0IG0gPSB2YWx1ZS5yb3dzO1xuICAgIGxldCBuID0gdmFsdWUuY29sdW1ucztcblxuICAgIGNvbnN0IHtcbiAgICAgIGNvbXB1dGVMZWZ0U2luZ3VsYXJWZWN0b3JzID0gdHJ1ZSxcbiAgICAgIGNvbXB1dGVSaWdodFNpbmd1bGFyVmVjdG9ycyA9IHRydWUsXG4gICAgICBhdXRvVHJhbnNwb3NlID0gZmFsc2UsXG4gICAgfSA9IG9wdGlvbnM7XG5cbiAgICBsZXQgd2FudHUgPSBCb29sZWFuKGNvbXB1dGVMZWZ0U2luZ3VsYXJWZWN0b3JzKTtcbiAgICBsZXQgd2FudHYgPSBCb29sZWFuKGNvbXB1dGVSaWdodFNpbmd1bGFyVmVjdG9ycyk7XG5cbiAgICBsZXQgc3dhcHBlZCA9IGZhbHNlO1xuICAgIGxldCBhO1xuICAgIGlmIChtIDwgbikge1xuICAgICAgaWYgKCFhdXRvVHJhbnNwb3NlKSB7XG4gICAgICAgIGEgPSB2YWx1ZS5jbG9uZSgpO1xuICAgICAgICAvLyBlc2xpbnQtZGlzYWJsZS1uZXh0LWxpbmUgbm8tY29uc29sZVxuICAgICAgICBjb25zb2xlLndhcm4oXG4gICAgICAgICAgJ0NvbXB1dGluZyBTVkQgb24gYSBtYXRyaXggd2l0aCBtb3JlIGNvbHVtbnMgdGhhbiByb3dzLiBDb25zaWRlciBlbmFibGluZyBhdXRvVHJhbnNwb3NlJyxcbiAgICAgICAgKTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIGEgPSB2YWx1ZS50cmFuc3Bvc2UoKTtcbiAgICAgICAgbSA9IGEucm93cztcbiAgICAgICAgbiA9IGEuY29sdW1ucztcbiAgICAgICAgc3dhcHBlZCA9IHRydWU7XG4gICAgICAgIGxldCBhdXggPSB3YW50dTtcbiAgICAgICAgd2FudHUgPSB3YW50djtcbiAgICAgICAgd2FudHYgPSBhdXg7XG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIGEgPSB2YWx1ZS5jbG9uZSgpO1xuICAgIH1cblxuICAgIGxldCBudSA9IE1hdGgubWluKG0sIG4pO1xuICAgIGxldCBuaSA9IE1hdGgubWluKG0gKyAxLCBuKTtcbiAgICBsZXQgcyA9IG5ldyBGbG9hdDY0QXJyYXkobmkpO1xuICAgIGxldCBVID0gbmV3IE1hdHJpeChtLCBudSk7XG4gICAgbGV0IFYgPSBuZXcgTWF0cml4KG4sIG4pO1xuXG4gICAgbGV0IGUgPSBuZXcgRmxvYXQ2NEFycmF5KG4pO1xuICAgIGxldCB3b3JrID0gbmV3IEZsb2F0NjRBcnJheShtKTtcblxuICAgIGxldCBzaSA9IG5ldyBGbG9hdDY0QXJyYXkobmkpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbmk7IGkrKykgc2lbaV0gPSBpO1xuXG4gICAgbGV0IG5jdCA9IE1hdGgubWluKG0gLSAxLCBuKTtcbiAgICBsZXQgbnJ0ID0gTWF0aC5tYXgoMCwgTWF0aC5taW4obiAtIDIsIG0pKTtcbiAgICBsZXQgbXJjID0gTWF0aC5tYXgobmN0LCBucnQpO1xuXG4gICAgZm9yIChsZXQgayA9IDA7IGsgPCBtcmM7IGsrKykge1xuICAgICAgaWYgKGsgPCBuY3QpIHtcbiAgICAgICAgc1trXSA9IDA7XG4gICAgICAgIGZvciAobGV0IGkgPSBrOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgc1trXSA9IGh5cG90ZW51c2Uoc1trXSwgYS5nZXQoaSwgaykpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChzW2tdICE9PSAwKSB7XG4gICAgICAgICAgaWYgKGEuZ2V0KGssIGspIDwgMCkge1xuICAgICAgICAgICAgc1trXSA9IC1zW2tdO1xuICAgICAgICAgIH1cbiAgICAgICAgICBmb3IgKGxldCBpID0gazsgaSA8IG07IGkrKykge1xuICAgICAgICAgICAgYS5zZXQoaSwgaywgYS5nZXQoaSwgaykgLyBzW2tdKTtcbiAgICAgICAgICB9XG4gICAgICAgICAgYS5zZXQoaywgaywgYS5nZXQoaywgaykgKyAxKTtcbiAgICAgICAgfVxuICAgICAgICBzW2tdID0gLXNba107XG4gICAgICB9XG5cbiAgICAgIGZvciAobGV0IGogPSBrICsgMTsgaiA8IG47IGorKykge1xuICAgICAgICBpZiAoayA8IG5jdCAmJiBzW2tdICE9PSAwKSB7XG4gICAgICAgICAgbGV0IHQgPSAwO1xuICAgICAgICAgIGZvciAobGV0IGkgPSBrOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICB0ICs9IGEuZ2V0KGksIGspICogYS5nZXQoaSwgaik7XG4gICAgICAgICAgfVxuICAgICAgICAgIHQgPSAtdCAvIGEuZ2V0KGssIGspO1xuICAgICAgICAgIGZvciAobGV0IGkgPSBrOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICBhLnNldChpLCBqLCBhLmdldChpLCBqKSArIHQgKiBhLmdldChpLCBrKSk7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIGVbal0gPSBhLmdldChrLCBqKTtcbiAgICAgIH1cblxuICAgICAgaWYgKHdhbnR1ICYmIGsgPCBuY3QpIHtcbiAgICAgICAgZm9yIChsZXQgaSA9IGs7IGkgPCBtOyBpKyspIHtcbiAgICAgICAgICBVLnNldChpLCBrLCBhLmdldChpLCBrKSk7XG4gICAgICAgIH1cbiAgICAgIH1cblxuICAgICAgaWYgKGsgPCBucnQpIHtcbiAgICAgICAgZVtrXSA9IDA7XG4gICAgICAgIGZvciAobGV0IGkgPSBrICsgMTsgaSA8IG47IGkrKykge1xuICAgICAgICAgIGVba10gPSBoeXBvdGVudXNlKGVba10sIGVbaV0pO1xuICAgICAgICB9XG4gICAgICAgIGlmIChlW2tdICE9PSAwKSB7XG4gICAgICAgICAgaWYgKGVbayArIDFdIDwgMCkge1xuICAgICAgICAgICAgZVtrXSA9IDAgLSBlW2tdO1xuICAgICAgICAgIH1cbiAgICAgICAgICBmb3IgKGxldCBpID0gayArIDE7IGkgPCBuOyBpKyspIHtcbiAgICAgICAgICAgIGVbaV0gLz0gZVtrXTtcbiAgICAgICAgICB9XG4gICAgICAgICAgZVtrICsgMV0gKz0gMTtcbiAgICAgICAgfVxuICAgICAgICBlW2tdID0gLWVba107XG4gICAgICAgIGlmIChrICsgMSA8IG0gJiYgZVtrXSAhPT0gMCkge1xuICAgICAgICAgIGZvciAobGV0IGkgPSBrICsgMTsgaSA8IG07IGkrKykge1xuICAgICAgICAgICAgd29ya1tpXSA9IDA7XG4gICAgICAgICAgfVxuICAgICAgICAgIGZvciAobGV0IGkgPSBrICsgMTsgaSA8IG07IGkrKykge1xuICAgICAgICAgICAgZm9yIChsZXQgaiA9IGsgKyAxOyBqIDwgbjsgaisrKSB7XG4gICAgICAgICAgICAgIHdvcmtbaV0gKz0gZVtqXSAqIGEuZ2V0KGksIGopO1xuICAgICAgICAgICAgfVxuICAgICAgICAgIH1cbiAgICAgICAgICBmb3IgKGxldCBqID0gayArIDE7IGogPCBuOyBqKyspIHtcbiAgICAgICAgICAgIGxldCB0ID0gLWVbal0gLyBlW2sgKyAxXTtcbiAgICAgICAgICAgIGZvciAobGV0IGkgPSBrICsgMTsgaSA8IG07IGkrKykge1xuICAgICAgICAgICAgICBhLnNldChpLCBqLCBhLmdldChpLCBqKSArIHQgKiB3b3JrW2ldKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgaWYgKHdhbnR2KSB7XG4gICAgICAgICAgZm9yIChsZXQgaSA9IGsgKyAxOyBpIDwgbjsgaSsrKSB7XG4gICAgICAgICAgICBWLnNldChpLCBrLCBlW2ldKTtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG5cbiAgICBsZXQgcCA9IE1hdGgubWluKG4sIG0gKyAxKTtcbiAgICBpZiAobmN0IDwgbikge1xuICAgICAgc1tuY3RdID0gYS5nZXQobmN0LCBuY3QpO1xuICAgIH1cbiAgICBpZiAobSA8IHApIHtcbiAgICAgIHNbcCAtIDFdID0gMDtcbiAgICB9XG4gICAgaWYgKG5ydCArIDEgPCBwKSB7XG4gICAgICBlW25ydF0gPSBhLmdldChucnQsIHAgLSAxKTtcbiAgICB9XG4gICAgZVtwIC0gMV0gPSAwO1xuXG4gICAgaWYgKHdhbnR1KSB7XG4gICAgICBmb3IgKGxldCBqID0gbmN0OyBqIDwgbnU7IGorKykge1xuICAgICAgICBmb3IgKGxldCBpID0gMDsgaSA8IG07IGkrKykge1xuICAgICAgICAgIFUuc2V0KGksIGosIDApO1xuICAgICAgICB9XG4gICAgICAgIFUuc2V0KGosIGosIDEpO1xuICAgICAgfVxuICAgICAgZm9yIChsZXQgayA9IG5jdCAtIDE7IGsgPj0gMDsgay0tKSB7XG4gICAgICAgIGlmIChzW2tdICE9PSAwKSB7XG4gICAgICAgICAgZm9yIChsZXQgaiA9IGsgKyAxOyBqIDwgbnU7IGorKykge1xuICAgICAgICAgICAgbGV0IHQgPSAwO1xuICAgICAgICAgICAgZm9yIChsZXQgaSA9IGs7IGkgPCBtOyBpKyspIHtcbiAgICAgICAgICAgICAgdCArPSBVLmdldChpLCBrKSAqIFUuZ2V0KGksIGopO1xuICAgICAgICAgICAgfVxuICAgICAgICAgICAgdCA9IC10IC8gVS5nZXQoaywgayk7XG4gICAgICAgICAgICBmb3IgKGxldCBpID0gazsgaSA8IG07IGkrKykge1xuICAgICAgICAgICAgICBVLnNldChpLCBqLCBVLmdldChpLCBqKSArIHQgKiBVLmdldChpLCBrKSk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICAgIGZvciAobGV0IGkgPSBrOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICBVLnNldChpLCBrLCAtVS5nZXQoaSwgaykpO1xuICAgICAgICAgIH1cbiAgICAgICAgICBVLnNldChrLCBrLCAxICsgVS5nZXQoaywgaykpO1xuICAgICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgayAtIDE7IGkrKykge1xuICAgICAgICAgICAgVS5zZXQoaSwgaywgMCk7XG4gICAgICAgICAgfVxuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICBVLnNldChpLCBrLCAwKTtcbiAgICAgICAgICB9XG4gICAgICAgICAgVS5zZXQoaywgaywgMSk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG5cbiAgICBpZiAod2FudHYpIHtcbiAgICAgIGZvciAobGV0IGsgPSBuIC0gMTsgayA+PSAwOyBrLS0pIHtcbiAgICAgICAgaWYgKGsgPCBucnQgJiYgZVtrXSAhPT0gMCkge1xuICAgICAgICAgIGZvciAobGV0IGogPSBrICsgMTsgaiA8IG47IGorKykge1xuICAgICAgICAgICAgbGV0IHQgPSAwO1xuICAgICAgICAgICAgZm9yIChsZXQgaSA9IGsgKyAxOyBpIDwgbjsgaSsrKSB7XG4gICAgICAgICAgICAgIHQgKz0gVi5nZXQoaSwgaykgKiBWLmdldChpLCBqKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIHQgPSAtdCAvIFYuZ2V0KGsgKyAxLCBrKTtcbiAgICAgICAgICAgIGZvciAobGV0IGkgPSBrICsgMTsgaSA8IG47IGkrKykge1xuICAgICAgICAgICAgICBWLnNldChpLCBqLCBWLmdldChpLCBqKSArIHQgKiBWLmdldChpLCBrKSk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbjsgaSsrKSB7XG4gICAgICAgICAgVi5zZXQoaSwgaywgMCk7XG4gICAgICAgIH1cbiAgICAgICAgVi5zZXQoaywgaywgMSk7XG4gICAgICB9XG4gICAgfVxuXG4gICAgbGV0IHBwID0gcCAtIDE7XG4gICAgbGV0IGl0ZXIgPSAwO1xuICAgIGxldCBlcHMgPSBOdW1iZXIuRVBTSUxPTjtcbiAgICB3aGlsZSAocCA+IDApIHtcbiAgICAgIGxldCBrLCBrYXNlO1xuICAgICAgZm9yIChrID0gcCAtIDI7IGsgPj0gLTE7IGstLSkge1xuICAgICAgICBpZiAoayA9PT0gLTEpIHtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgICBjb25zdCBhbHBoYSA9XG4gICAgICAgICAgTnVtYmVyLk1JTl9WQUxVRSArIGVwcyAqIE1hdGguYWJzKHNba10gKyBNYXRoLmFicyhzW2sgKyAxXSkpO1xuICAgICAgICBpZiAoTWF0aC5hYnMoZVtrXSkgPD0gYWxwaGEgfHwgTnVtYmVyLmlzTmFOKGVba10pKSB7XG4gICAgICAgICAgZVtrXSA9IDA7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIGlmIChrID09PSBwIC0gMikge1xuICAgICAgICBrYXNlID0gNDtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIGxldCBrcztcbiAgICAgICAgZm9yIChrcyA9IHAgLSAxOyBrcyA+PSBrOyBrcy0tKSB7XG4gICAgICAgICAgaWYgKGtzID09PSBrKSB7XG4gICAgICAgICAgICBicmVhaztcbiAgICAgICAgICB9XG4gICAgICAgICAgbGV0IHQgPVxuICAgICAgICAgICAgKGtzICE9PSBwID8gTWF0aC5hYnMoZVtrc10pIDogMCkgK1xuICAgICAgICAgICAgKGtzICE9PSBrICsgMSA/IE1hdGguYWJzKGVba3MgLSAxXSkgOiAwKTtcbiAgICAgICAgICBpZiAoTWF0aC5hYnMoc1trc10pIDw9IGVwcyAqIHQpIHtcbiAgICAgICAgICAgIHNba3NdID0gMDtcbiAgICAgICAgICAgIGJyZWFrO1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICBpZiAoa3MgPT09IGspIHtcbiAgICAgICAgICBrYXNlID0gMztcbiAgICAgICAgfSBlbHNlIGlmIChrcyA9PT0gcCAtIDEpIHtcbiAgICAgICAgICBrYXNlID0gMTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICBrYXNlID0gMjtcbiAgICAgICAgICBrID0ga3M7XG4gICAgICAgIH1cbiAgICAgIH1cblxuICAgICAgaysrO1xuXG4gICAgICBzd2l0Y2ggKGthc2UpIHtcbiAgICAgICAgY2FzZSAxOiB7XG4gICAgICAgICAgbGV0IGYgPSBlW3AgLSAyXTtcbiAgICAgICAgICBlW3AgLSAyXSA9IDA7XG4gICAgICAgICAgZm9yIChsZXQgaiA9IHAgLSAyOyBqID49IGs7IGotLSkge1xuICAgICAgICAgICAgbGV0IHQgPSBoeXBvdGVudXNlKHNbal0sIGYpO1xuICAgICAgICAgICAgbGV0IGNzID0gc1tqXSAvIHQ7XG4gICAgICAgICAgICBsZXQgc24gPSBmIC8gdDtcbiAgICAgICAgICAgIHNbal0gPSB0O1xuICAgICAgICAgICAgaWYgKGogIT09IGspIHtcbiAgICAgICAgICAgICAgZiA9IC1zbiAqIGVbaiAtIDFdO1xuICAgICAgICAgICAgICBlW2ogLSAxXSA9IGNzICogZVtqIC0gMV07XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpZiAod2FudHYpIHtcbiAgICAgICAgICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCBuOyBpKyspIHtcbiAgICAgICAgICAgICAgICB0ID0gY3MgKiBWLmdldChpLCBqKSArIHNuICogVi5nZXQoaSwgcCAtIDEpO1xuICAgICAgICAgICAgICAgIFYuc2V0KGksIHAgLSAxLCAtc24gKiBWLmdldChpLCBqKSArIGNzICogVi5nZXQoaSwgcCAtIDEpKTtcbiAgICAgICAgICAgICAgICBWLnNldChpLCBqLCB0KTtcbiAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuICAgICAgICAgIH1cbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgICBjYXNlIDI6IHtcbiAgICAgICAgICBsZXQgZiA9IGVbayAtIDFdO1xuICAgICAgICAgIGVbayAtIDFdID0gMDtcbiAgICAgICAgICBmb3IgKGxldCBqID0gazsgaiA8IHA7IGorKykge1xuICAgICAgICAgICAgbGV0IHQgPSBoeXBvdGVudXNlKHNbal0sIGYpO1xuICAgICAgICAgICAgbGV0IGNzID0gc1tqXSAvIHQ7XG4gICAgICAgICAgICBsZXQgc24gPSBmIC8gdDtcbiAgICAgICAgICAgIHNbal0gPSB0O1xuICAgICAgICAgICAgZiA9IC1zbiAqIGVbal07XG4gICAgICAgICAgICBlW2pdID0gY3MgKiBlW2pdO1xuICAgICAgICAgICAgaWYgKHdhbnR1KSB7XG4gICAgICAgICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICAgICAgdCA9IGNzICogVS5nZXQoaSwgaikgKyBzbiAqIFUuZ2V0KGksIGsgLSAxKTtcbiAgICAgICAgICAgICAgICBVLnNldChpLCBrIC0gMSwgLXNuICogVS5nZXQoaSwgaikgKyBjcyAqIFUuZ2V0KGksIGsgLSAxKSk7XG4gICAgICAgICAgICAgICAgVS5zZXQoaSwgaiwgdCk7XG4gICAgICAgICAgICAgIH1cbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIH1cbiAgICAgICAgY2FzZSAzOiB7XG4gICAgICAgICAgY29uc3Qgc2NhbGUgPSBNYXRoLm1heChcbiAgICAgICAgICAgIE1hdGguYWJzKHNbcCAtIDFdKSxcbiAgICAgICAgICAgIE1hdGguYWJzKHNbcCAtIDJdKSxcbiAgICAgICAgICAgIE1hdGguYWJzKGVbcCAtIDJdKSxcbiAgICAgICAgICAgIE1hdGguYWJzKHNba10pLFxuICAgICAgICAgICAgTWF0aC5hYnMoZVtrXSksXG4gICAgICAgICAgKTtcbiAgICAgICAgICBjb25zdCBzcCA9IHNbcCAtIDFdIC8gc2NhbGU7XG4gICAgICAgICAgY29uc3Qgc3BtMSA9IHNbcCAtIDJdIC8gc2NhbGU7XG4gICAgICAgICAgY29uc3QgZXBtMSA9IGVbcCAtIDJdIC8gc2NhbGU7XG4gICAgICAgICAgY29uc3Qgc2sgPSBzW2tdIC8gc2NhbGU7XG4gICAgICAgICAgY29uc3QgZWsgPSBlW2tdIC8gc2NhbGU7XG4gICAgICAgICAgY29uc3QgYiA9ICgoc3BtMSArIHNwKSAqIChzcG0xIC0gc3ApICsgZXBtMSAqIGVwbTEpIC8gMjtcbiAgICAgICAgICBjb25zdCBjID0gc3AgKiBlcG0xICogKHNwICogZXBtMSk7XG4gICAgICAgICAgbGV0IHNoaWZ0ID0gMDtcbiAgICAgICAgICBpZiAoYiAhPT0gMCB8fCBjICE9PSAwKSB7XG4gICAgICAgICAgICBpZiAoYiA8IDApIHtcbiAgICAgICAgICAgICAgc2hpZnQgPSAwIC0gTWF0aC5zcXJ0KGIgKiBiICsgYyk7XG4gICAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgICBzaGlmdCA9IE1hdGguc3FydChiICogYiArIGMpO1xuICAgICAgICAgICAgfVxuICAgICAgICAgICAgc2hpZnQgPSBjIC8gKGIgKyBzaGlmdCk7XG4gICAgICAgICAgfVxuICAgICAgICAgIGxldCBmID0gKHNrICsgc3ApICogKHNrIC0gc3ApICsgc2hpZnQ7XG4gICAgICAgICAgbGV0IGcgPSBzayAqIGVrO1xuICAgICAgICAgIGZvciAobGV0IGogPSBrOyBqIDwgcCAtIDE7IGorKykge1xuICAgICAgICAgICAgbGV0IHQgPSBoeXBvdGVudXNlKGYsIGcpO1xuICAgICAgICAgICAgaWYgKHQgPT09IDApIHQgPSBOdW1iZXIuTUlOX1ZBTFVFO1xuICAgICAgICAgICAgbGV0IGNzID0gZiAvIHQ7XG4gICAgICAgICAgICBsZXQgc24gPSBnIC8gdDtcbiAgICAgICAgICAgIGlmIChqICE9PSBrKSB7XG4gICAgICAgICAgICAgIGVbaiAtIDFdID0gdDtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIGYgPSBjcyAqIHNbal0gKyBzbiAqIGVbal07XG4gICAgICAgICAgICBlW2pdID0gY3MgKiBlW2pdIC0gc24gKiBzW2pdO1xuICAgICAgICAgICAgZyA9IHNuICogc1tqICsgMV07XG4gICAgICAgICAgICBzW2ogKyAxXSA9IGNzICogc1tqICsgMV07XG4gICAgICAgICAgICBpZiAod2FudHYpIHtcbiAgICAgICAgICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCBuOyBpKyspIHtcbiAgICAgICAgICAgICAgICB0ID0gY3MgKiBWLmdldChpLCBqKSArIHNuICogVi5nZXQoaSwgaiArIDEpO1xuICAgICAgICAgICAgICAgIFYuc2V0KGksIGogKyAxLCAtc24gKiBWLmdldChpLCBqKSArIGNzICogVi5nZXQoaSwgaiArIDEpKTtcbiAgICAgICAgICAgICAgICBWLnNldChpLCBqLCB0KTtcbiAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuICAgICAgICAgICAgdCA9IGh5cG90ZW51c2UoZiwgZyk7XG4gICAgICAgICAgICBpZiAodCA9PT0gMCkgdCA9IE51bWJlci5NSU5fVkFMVUU7XG4gICAgICAgICAgICBjcyA9IGYgLyB0O1xuICAgICAgICAgICAgc24gPSBnIC8gdDtcbiAgICAgICAgICAgIHNbal0gPSB0O1xuICAgICAgICAgICAgZiA9IGNzICogZVtqXSArIHNuICogc1tqICsgMV07XG4gICAgICAgICAgICBzW2ogKyAxXSA9IC1zbiAqIGVbal0gKyBjcyAqIHNbaiArIDFdO1xuICAgICAgICAgICAgZyA9IHNuICogZVtqICsgMV07XG4gICAgICAgICAgICBlW2ogKyAxXSA9IGNzICogZVtqICsgMV07XG4gICAgICAgICAgICBpZiAod2FudHUgJiYgaiA8IG0gLSAxKSB7XG4gICAgICAgICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICAgICAgdCA9IGNzICogVS5nZXQoaSwgaikgKyBzbiAqIFUuZ2V0KGksIGogKyAxKTtcbiAgICAgICAgICAgICAgICBVLnNldChpLCBqICsgMSwgLXNuICogVS5nZXQoaSwgaikgKyBjcyAqIFUuZ2V0KGksIGogKyAxKSk7XG4gICAgICAgICAgICAgICAgVS5zZXQoaSwgaiwgdCk7XG4gICAgICAgICAgICAgIH1cbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG4gICAgICAgICAgZVtwIC0gMl0gPSBmO1xuICAgICAgICAgIGl0ZXIgPSBpdGVyICsgMTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgICBjYXNlIDQ6IHtcbiAgICAgICAgICBpZiAoc1trXSA8PSAwKSB7XG4gICAgICAgICAgICBzW2tdID0gc1trXSA8IDAgPyAtc1trXSA6IDA7XG4gICAgICAgICAgICBpZiAod2FudHYpIHtcbiAgICAgICAgICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPD0gcHA7IGkrKykge1xuICAgICAgICAgICAgICAgIFYuc2V0KGksIGssIC1WLmdldChpLCBrKSk7XG4gICAgICAgICAgICAgIH1cbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG4gICAgICAgICAgd2hpbGUgKGsgPCBwcCkge1xuICAgICAgICAgICAgaWYgKHNba10gPj0gc1trICsgMV0pIHtcbiAgICAgICAgICAgICAgYnJlYWs7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBsZXQgdCA9IHNba107XG4gICAgICAgICAgICBzW2tdID0gc1trICsgMV07XG4gICAgICAgICAgICBzW2sgKyAxXSA9IHQ7XG4gICAgICAgICAgICBpZiAod2FudHYgJiYgayA8IG4gLSAxKSB7XG4gICAgICAgICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbjsgaSsrKSB7XG4gICAgICAgICAgICAgICAgdCA9IFYuZ2V0KGksIGsgKyAxKTtcbiAgICAgICAgICAgICAgICBWLnNldChpLCBrICsgMSwgVi5nZXQoaSwgaykpO1xuICAgICAgICAgICAgICAgIFYuc2V0KGksIGssIHQpO1xuICAgICAgICAgICAgICB9XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpZiAod2FudHUgJiYgayA8IG0gLSAxKSB7XG4gICAgICAgICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgICAgICAgICAgdCA9IFUuZ2V0KGksIGsgKyAxKTtcbiAgICAgICAgICAgICAgICBVLnNldChpLCBrICsgMSwgVS5nZXQoaSwgaykpO1xuICAgICAgICAgICAgICAgIFUuc2V0KGksIGssIHQpO1xuICAgICAgICAgICAgICB9XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBrKys7XG4gICAgICAgICAgfVxuICAgICAgICAgIGl0ZXIgPSAwO1xuICAgICAgICAgIHAtLTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgICAvLyBubyBkZWZhdWx0XG4gICAgICB9XG4gICAgfVxuXG4gICAgaWYgKHN3YXBwZWQpIHtcbiAgICAgIGxldCB0bXAgPSBWO1xuICAgICAgViA9IFU7XG4gICAgICBVID0gdG1wO1xuICAgIH1cblxuICAgIHRoaXMubSA9IG07XG4gICAgdGhpcy5uID0gbjtcbiAgICB0aGlzLnMgPSBzO1xuICAgIHRoaXMuVSA9IFU7XG4gICAgdGhpcy5WID0gVjtcbiAgfVxuXG4gIHNvbHZlKHZhbHVlKSB7XG4gICAgbGV0IFkgPSB2YWx1ZTtcbiAgICBsZXQgZSA9IHRoaXMudGhyZXNob2xkO1xuICAgIGxldCBzY29scyA9IHRoaXMucy5sZW5ndGg7XG4gICAgbGV0IExzID0gTWF0cml4Lnplcm9zKHNjb2xzLCBzY29scyk7XG5cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHNjb2xzOyBpKyspIHtcbiAgICAgIGlmIChNYXRoLmFicyh0aGlzLnNbaV0pIDw9IGUpIHtcbiAgICAgICAgTHMuc2V0KGksIGksIDApO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgTHMuc2V0KGksIGksIDEgLyB0aGlzLnNbaV0pO1xuICAgICAgfVxuICAgIH1cblxuICAgIGxldCBVID0gdGhpcy5VO1xuICAgIGxldCBWID0gdGhpcy5yaWdodFNpbmd1bGFyVmVjdG9ycztcblxuICAgIGxldCBWTCA9IFYubW11bChMcyk7XG4gICAgbGV0IHZyb3dzID0gVi5yb3dzO1xuICAgIGxldCB1cm93cyA9IFUucm93cztcbiAgICBsZXQgVkxVID0gTWF0cml4Lnplcm9zKHZyb3dzLCB1cm93cyk7XG5cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHZyb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdXJvd3M7IGorKykge1xuICAgICAgICBsZXQgc3VtID0gMDtcbiAgICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCBzY29sczsgaysrKSB7XG4gICAgICAgICAgc3VtICs9IFZMLmdldChpLCBrKSAqIFUuZ2V0KGosIGspO1xuICAgICAgICB9XG4gICAgICAgIFZMVS5zZXQoaSwgaiwgc3VtKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICByZXR1cm4gVkxVLm1tdWwoWSk7XG4gIH1cblxuICBzb2x2ZUZvckRpYWdvbmFsKHZhbHVlKSB7XG4gICAgcmV0dXJuIHRoaXMuc29sdmUoTWF0cml4LmRpYWcodmFsdWUpKTtcbiAgfVxuXG4gIGludmVyc2UoKSB7XG4gICAgbGV0IFYgPSB0aGlzLlY7XG4gICAgbGV0IGUgPSB0aGlzLnRocmVzaG9sZDtcbiAgICBsZXQgdnJvd3MgPSBWLnJvd3M7XG4gICAgbGV0IHZjb2xzID0gVi5jb2x1bW5zO1xuICAgIGxldCBYID0gbmV3IE1hdHJpeCh2cm93cywgdGhpcy5zLmxlbmd0aCk7XG5cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHZyb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdmNvbHM7IGorKykge1xuICAgICAgICBpZiAoTWF0aC5hYnModGhpcy5zW2pdKSA+IGUpIHtcbiAgICAgICAgICBYLnNldChpLCBqLCBWLmdldChpLCBqKSAvIHRoaXMuc1tqXSk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG5cbiAgICBsZXQgVSA9IHRoaXMuVTtcblxuICAgIGxldCB1cm93cyA9IFUucm93cztcbiAgICBsZXQgdWNvbHMgPSBVLmNvbHVtbnM7XG4gICAgbGV0IFkgPSBuZXcgTWF0cml4KHZyb3dzLCB1cm93cyk7XG5cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHZyb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdXJvd3M7IGorKykge1xuICAgICAgICBsZXQgc3VtID0gMDtcbiAgICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCB1Y29sczsgaysrKSB7XG4gICAgICAgICAgc3VtICs9IFguZ2V0KGksIGspICogVS5nZXQoaiwgayk7XG4gICAgICAgIH1cbiAgICAgICAgWS5zZXQoaSwgaiwgc3VtKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICByZXR1cm4gWTtcbiAgfVxuXG4gIGdldCBjb25kaXRpb24oKSB7XG4gICAgcmV0dXJuIHRoaXMuc1swXSAvIHRoaXMuc1tNYXRoLm1pbih0aGlzLm0sIHRoaXMubikgLSAxXTtcbiAgfVxuXG4gIGdldCBub3JtMigpIHtcbiAgICByZXR1cm4gdGhpcy5zWzBdO1xuICB9XG5cbiAgZ2V0IHJhbmsoKSB7XG4gICAgbGV0IHRvbCA9IE1hdGgubWF4KHRoaXMubSwgdGhpcy5uKSAqIHRoaXMuc1swXSAqIE51bWJlci5FUFNJTE9OO1xuICAgIGxldCByID0gMDtcbiAgICBsZXQgcyA9IHRoaXMucztcbiAgICBmb3IgKGxldCBpID0gMCwgaWkgPSBzLmxlbmd0aDsgaSA8IGlpOyBpKyspIHtcbiAgICAgIGlmIChzW2ldID4gdG9sKSB7XG4gICAgICAgIHIrKztcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHI7XG4gIH1cblxuICBnZXQgZGlhZ29uYWwoKSB7XG4gICAgcmV0dXJuIEFycmF5LmZyb20odGhpcy5zKTtcbiAgfVxuXG4gIGdldCB0aHJlc2hvbGQoKSB7XG4gICAgcmV0dXJuIChOdW1iZXIuRVBTSUxPTiAvIDIpICogTWF0aC5tYXgodGhpcy5tLCB0aGlzLm4pICogdGhpcy5zWzBdO1xuICB9XG5cbiAgZ2V0IGxlZnRTaW5ndWxhclZlY3RvcnMoKSB7XG4gICAgcmV0dXJuIHRoaXMuVTtcbiAgfVxuXG4gIGdldCByaWdodFNpbmd1bGFyVmVjdG9ycygpIHtcbiAgICByZXR1cm4gdGhpcy5WO1xuICB9XG5cbiAgZ2V0IGRpYWdvbmFsTWF0cml4KCkge1xuICAgIHJldHVybiBNYXRyaXguZGlhZyh0aGlzLnMpO1xuICB9XG59XG4iLCJleHBvcnQgZnVuY3Rpb24gaHlwb3RlbnVzZShhLCBiKSB7XG4gIGxldCByID0gMDtcbiAgaWYgKE1hdGguYWJzKGEpID4gTWF0aC5hYnMoYikpIHtcbiAgICByID0gYiAvIGE7XG4gICAgcmV0dXJuIE1hdGguYWJzKGEpICogTWF0aC5zcXJ0KDEgKyByICogcik7XG4gIH1cbiAgaWYgKGIgIT09IDApIHtcbiAgICByID0gYSAvIGI7XG4gICAgcmV0dXJuIE1hdGguYWJzKGIpICogTWF0aC5zcXJ0KDEgKyByICogcik7XG4gIH1cbiAgcmV0dXJuIDA7XG59XG4iLCJpbXBvcnQgTHVEZWNvbXBvc2l0aW9uIGZyb20gJy4vZGMvbHUnO1xuaW1wb3J0IFFyRGVjb21wb3NpdGlvbiBmcm9tICcuL2RjL3FyJztcbmltcG9ydCBTaW5ndWxhclZhbHVlRGVjb21wb3NpdGlvbiBmcm9tICcuL2RjL3N2ZCc7XG5pbXBvcnQgTWF0cml4IGZyb20gJy4vbWF0cml4JztcbmltcG9ydCBXcmFwcGVyTWF0cml4MkQgZnJvbSAnLi93cmFwL1dyYXBwZXJNYXRyaXgyRCc7XG5cbmV4cG9ydCBmdW5jdGlvbiBpbnZlcnNlKG1hdHJpeCwgdXNlU1ZEID0gZmFsc2UpIHtcbiAgbWF0cml4ID0gV3JhcHBlck1hdHJpeDJELmNoZWNrTWF0cml4KG1hdHJpeCk7XG4gIGlmICh1c2VTVkQpIHtcbiAgICByZXR1cm4gbmV3IFNpbmd1bGFyVmFsdWVEZWNvbXBvc2l0aW9uKG1hdHJpeCkuaW52ZXJzZSgpO1xuICB9IGVsc2Uge1xuICAgIHJldHVybiBzb2x2ZShtYXRyaXgsIE1hdHJpeC5leWUobWF0cml4LnJvd3MpKTtcbiAgfVxufVxuXG5leHBvcnQgZnVuY3Rpb24gc29sdmUobGVmdEhhbmRTaWRlLCByaWdodEhhbmRTaWRlLCB1c2VTVkQgPSBmYWxzZSkge1xuICBsZWZ0SGFuZFNpZGUgPSBXcmFwcGVyTWF0cml4MkQuY2hlY2tNYXRyaXgobGVmdEhhbmRTaWRlKTtcbiAgcmlnaHRIYW5kU2lkZSA9IFdyYXBwZXJNYXRyaXgyRC5jaGVja01hdHJpeChyaWdodEhhbmRTaWRlKTtcbiAgaWYgKHVzZVNWRCkge1xuICAgIHJldHVybiBuZXcgU2luZ3VsYXJWYWx1ZURlY29tcG9zaXRpb24obGVmdEhhbmRTaWRlKS5zb2x2ZShyaWdodEhhbmRTaWRlKTtcbiAgfSBlbHNlIHtcbiAgICByZXR1cm4gbGVmdEhhbmRTaWRlLmlzU3F1YXJlKClcbiAgICAgID8gbmV3IEx1RGVjb21wb3NpdGlvbihsZWZ0SGFuZFNpZGUpLnNvbHZlKHJpZ2h0SGFuZFNpZGUpXG4gICAgICA6IG5ldyBRckRlY29tcG9zaXRpb24obGVmdEhhbmRTaWRlKS5zb2x2ZShyaWdodEhhbmRTaWRlKTtcbiAgfVxufVxuIiwiaW1wb3J0IEx1RGVjb21wb3NpdGlvbiBmcm9tICcuL2RjL2x1JztcbmltcG9ydCBNYXRyaXggZnJvbSAnLi9tYXRyaXgnO1xuaW1wb3J0IE1hdHJpeFNlbGVjdGlvblZpZXcgZnJvbSAnLi92aWV3cy9zZWxlY3Rpb24nO1xuXG5leHBvcnQgZnVuY3Rpb24gZGV0ZXJtaW5hbnQobWF0cml4KSB7XG4gIG1hdHJpeCA9IE1hdHJpeC5jaGVja01hdHJpeChtYXRyaXgpO1xuICBpZiAobWF0cml4LmlzU3F1YXJlKCkpIHtcbiAgICBpZiAobWF0cml4LmNvbHVtbnMgPT09IDApIHtcbiAgICAgIHJldHVybiAxO1xuICAgIH1cblxuICAgIGxldCBhLCBiLCBjLCBkO1xuICAgIGlmIChtYXRyaXguY29sdW1ucyA9PT0gMikge1xuICAgICAgLy8gMiB4IDIgbWF0cml4XG4gICAgICBhID0gbWF0cml4LmdldCgwLCAwKTtcbiAgICAgIGIgPSBtYXRyaXguZ2V0KDAsIDEpO1xuICAgICAgYyA9IG1hdHJpeC5nZXQoMSwgMCk7XG4gICAgICBkID0gbWF0cml4LmdldCgxLCAxKTtcblxuICAgICAgcmV0dXJuIGEgKiBkIC0gYiAqIGM7XG4gICAgfSBlbHNlIGlmIChtYXRyaXguY29sdW1ucyA9PT0gMykge1xuICAgICAgLy8gMyB4IDMgbWF0cml4XG4gICAgICBsZXQgc3ViTWF0cml4MCwgc3ViTWF0cml4MSwgc3ViTWF0cml4MjtcbiAgICAgIHN1Yk1hdHJpeDAgPSBuZXcgTWF0cml4U2VsZWN0aW9uVmlldyhtYXRyaXgsIFsxLCAyXSwgWzEsIDJdKTtcbiAgICAgIHN1Yk1hdHJpeDEgPSBuZXcgTWF0cml4U2VsZWN0aW9uVmlldyhtYXRyaXgsIFsxLCAyXSwgWzAsIDJdKTtcbiAgICAgIHN1Yk1hdHJpeDIgPSBuZXcgTWF0cml4U2VsZWN0aW9uVmlldyhtYXRyaXgsIFsxLCAyXSwgWzAsIDFdKTtcbiAgICAgIGEgPSBtYXRyaXguZ2V0KDAsIDApO1xuICAgICAgYiA9IG1hdHJpeC5nZXQoMCwgMSk7XG4gICAgICBjID0gbWF0cml4LmdldCgwLCAyKTtcblxuICAgICAgcmV0dXJuIChcbiAgICAgICAgYSAqIGRldGVybWluYW50KHN1Yk1hdHJpeDApIC1cbiAgICAgICAgYiAqIGRldGVybWluYW50KHN1Yk1hdHJpeDEpICtcbiAgICAgICAgYyAqIGRldGVybWluYW50KHN1Yk1hdHJpeDIpXG4gICAgICApO1xuICAgIH0gZWxzZSB7XG4gICAgICAvLyBnZW5lcmFsIHB1cnBvc2UgZGV0ZXJtaW5hbnQgdXNpbmcgdGhlIExVIGRlY29tcG9zaXRpb25cbiAgICAgIHJldHVybiBuZXcgTHVEZWNvbXBvc2l0aW9uKG1hdHJpeCkuZGV0ZXJtaW5hbnQ7XG4gICAgfVxuICB9IGVsc2Uge1xuICAgIHRocm93IEVycm9yKCdkZXRlcm1pbmFudCBjYW4gb25seSBiZSBjYWxjdWxhdGVkIGZvciBhIHNxdWFyZSBtYXRyaXgnKTtcbiAgfVxufVxuIiwiZXhwb3J0IHsgQWJzdHJhY3RNYXRyaXgsIGRlZmF1bHQsIGRlZmF1bHQgYXMgTWF0cml4IH0gZnJvbSAnLi9tYXRyaXgnO1xuZXhwb3J0ICogZnJvbSAnLi92aWV3cy9pbmRleCc7XG5cbmV4cG9ydCB7IHdyYXAgfSBmcm9tICcuL3dyYXAvd3JhcCc7XG5leHBvcnQgeyBkZWZhdWx0IGFzIFdyYXBwZXJNYXRyaXgxRCB9IGZyb20gJy4vd3JhcC9XcmFwcGVyTWF0cml4MUQnO1xuZXhwb3J0IHsgZGVmYXVsdCBhcyBXcmFwcGVyTWF0cml4MkQgfSBmcm9tICcuL3dyYXAvV3JhcHBlck1hdHJpeDJEJztcblxuZXhwb3J0IHsgc29sdmUsIGludmVyc2UgfSBmcm9tICcuL2RlY29tcG9zaXRpb25zJztcbmV4cG9ydCB7IGRldGVybWluYW50IH0gZnJvbSAnLi9kZXRlcm1pbmFudCc7XG5leHBvcnQgeyBsaW5lYXJEZXBlbmRlbmNpZXMgfSBmcm9tICcuL2xpbmVhckRlcGVuZGVuY2llcyc7XG5leHBvcnQgeyBwc2V1ZG9JbnZlcnNlIH0gZnJvbSAnLi9wc2V1ZG9JbnZlcnNlJztcbmV4cG9ydCB7IGNvdmFyaWFuY2UgfSBmcm9tICcuL2NvdmFyaWFuY2UnO1xuZXhwb3J0IHsgY29ycmVsYXRpb24gfSBmcm9tICcuL2NvcnJlbGF0aW9uJztcblxuZXhwb3J0IHtcbiAgZGVmYXVsdCBhcyBTaW5ndWxhclZhbHVlRGVjb21wb3NpdGlvbixcbiAgZGVmYXVsdCBhcyBTVkQsXG59IGZyb20gJy4vZGMvc3ZkLmpzJztcbmV4cG9ydCB7XG4gIGRlZmF1bHQgYXMgRWlnZW52YWx1ZURlY29tcG9zaXRpb24sXG4gIGRlZmF1bHQgYXMgRVZELFxufSBmcm9tICcuL2RjL2V2ZC5qcyc7XG5leHBvcnQge1xuICBkZWZhdWx0IGFzIENob2xlc2t5RGVjb21wb3NpdGlvbixcbiAgZGVmYXVsdCBhcyBDSE8sXG59IGZyb20gJy4vZGMvY2hvbGVza3kuanMnO1xuZXhwb3J0IHsgZGVmYXVsdCBhcyBMdURlY29tcG9zaXRpb24sIGRlZmF1bHQgYXMgTFUgfSBmcm9tICcuL2RjL2x1LmpzJztcbmV4cG9ydCB7IGRlZmF1bHQgYXMgUXJEZWNvbXBvc2l0aW9uLCBkZWZhdWx0IGFzIFFSIH0gZnJvbSAnLi9kYy9xci5qcyc7XG5leHBvcnQgeyBkZWZhdWx0IGFzIE5pcGFscywgZGVmYXVsdCBhcyBOSVBBTFMgfSBmcm9tICcuL2RjL25pcGFscy5qcyc7XG4iLCJjb25zdCBpbmRlbnQgPSAnICcucmVwZWF0KDIpO1xuY29uc3QgaW5kZW50RGF0YSA9ICcgJy5yZXBlYXQoNCk7XG5cbmV4cG9ydCBmdW5jdGlvbiBpbnNwZWN0TWF0cml4KCkge1xuICByZXR1cm4gaW5zcGVjdE1hdHJpeFdpdGhPcHRpb25zKHRoaXMpO1xufVxuXG5leHBvcnQgZnVuY3Rpb24gaW5zcGVjdE1hdHJpeFdpdGhPcHRpb25zKG1hdHJpeCwgb3B0aW9ucyA9IHt9KSB7XG4gIGNvbnN0IHtcbiAgICBtYXhSb3dzID0gMTUsXG4gICAgbWF4Q29sdW1ucyA9IDEwLFxuICAgIG1heE51bVNpemUgPSA4LFxuICAgIHBhZE1pbnVzID0gJ2F1dG8nLFxuICB9ID0gb3B0aW9ucztcbiAgcmV0dXJuIGAke21hdHJpeC5jb25zdHJ1Y3Rvci5uYW1lfSB7XG4ke2luZGVudH1bXG4ke2luZGVudERhdGF9JHtpbnNwZWN0RGF0YShtYXRyaXgsIG1heFJvd3MsIG1heENvbHVtbnMsIG1heE51bVNpemUsIHBhZE1pbnVzKX1cbiR7aW5kZW50fV1cbiR7aW5kZW50fXJvd3M6ICR7bWF0cml4LnJvd3N9XG4ke2luZGVudH1jb2x1bW5zOiAke21hdHJpeC5jb2x1bW5zfVxufWA7XG59XG5cbmZ1bmN0aW9uIGluc3BlY3REYXRhKG1hdHJpeCwgbWF4Um93cywgbWF4Q29sdW1ucywgbWF4TnVtU2l6ZSwgcGFkTWludXMpIHtcbiAgY29uc3QgeyByb3dzLCBjb2x1bW5zIH0gPSBtYXRyaXg7XG4gIGNvbnN0IG1heEkgPSBNYXRoLm1pbihyb3dzLCBtYXhSb3dzKTtcbiAgY29uc3QgbWF4SiA9IE1hdGgubWluKGNvbHVtbnMsIG1heENvbHVtbnMpO1xuICBjb25zdCByZXN1bHQgPSBbXTtcblxuICBpZiAocGFkTWludXMgPT09ICdhdXRvJykge1xuICAgIHBhZE1pbnVzID0gZmFsc2U7XG4gICAgbG9vcDogZm9yIChsZXQgaSA9IDA7IGkgPCBtYXhJOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgbWF4SjsgaisrKSB7XG4gICAgICAgIGlmIChtYXRyaXguZ2V0KGksIGopIDwgMCkge1xuICAgICAgICAgIHBhZE1pbnVzID0gdHJ1ZTtcbiAgICAgICAgICBicmVhayBsb29wO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXhJOyBpKyspIHtcbiAgICBsZXQgbGluZSA9IFtdO1xuICAgIGZvciAobGV0IGogPSAwOyBqIDwgbWF4SjsgaisrKSB7XG4gICAgICBsaW5lLnB1c2goZm9ybWF0TnVtYmVyKG1hdHJpeC5nZXQoaSwgaiksIG1heE51bVNpemUsIHBhZE1pbnVzKSk7XG4gICAgfVxuICAgIHJlc3VsdC5wdXNoKGAke2xpbmUuam9pbignICcpfWApO1xuICB9XG4gIGlmIChtYXhKICE9PSBjb2x1bW5zKSB7XG4gICAgcmVzdWx0W3Jlc3VsdC5sZW5ndGggLSAxXSArPSBgIC4uLiAke2NvbHVtbnMgLSBtYXhDb2x1bW5zfSBtb3JlIGNvbHVtbnNgO1xuICB9XG4gIGlmIChtYXhJICE9PSByb3dzKSB7XG4gICAgcmVzdWx0LnB1c2goYC4uLiAke3Jvd3MgLSBtYXhSb3dzfSBtb3JlIHJvd3NgKTtcbiAgfVxuICByZXR1cm4gcmVzdWx0LmpvaW4oYFxcbiR7aW5kZW50RGF0YX1gKTtcbn1cblxuZnVuY3Rpb24gZm9ybWF0TnVtYmVyKG51bSwgbWF4TnVtU2l6ZSwgcGFkTWludXMpIHtcbiAgcmV0dXJuIChcbiAgICBudW0gPj0gMCAmJiBwYWRNaW51c1xuICAgICAgPyBgICR7Zm9ybWF0TnVtYmVyMihudW0sIG1heE51bVNpemUgLSAxKX1gXG4gICAgICA6IGZvcm1hdE51bWJlcjIobnVtLCBtYXhOdW1TaXplKVxuICApLnBhZEVuZChtYXhOdW1TaXplKTtcbn1cblxuZnVuY3Rpb24gZm9ybWF0TnVtYmVyMihudW0sIGxlbikge1xuICAvLyBzbWFsbC5sZW5ndGggbnVtYmVycyBzaG91bGQgYmUgYXMgaXNcbiAgbGV0IHN0ciA9IG51bS50b1N0cmluZygpO1xuICBpZiAoc3RyLmxlbmd0aCA8PSBsZW4pIHJldHVybiBzdHI7XG5cbiAgLy8gKDcpJzAuMDAxMjMnIGlzIGJldHRlciB0aGVuICg3KScxLjIzZS0yJ1xuICAvLyAoOCknMC4wMDAxMjMnIGlzIHdvcnNlIHRoZW4gKDcpJzEuMjNlLTMnLFxuICBsZXQgZml4ID0gbnVtLnRvRml4ZWQobGVuKTtcbiAgaWYgKGZpeC5sZW5ndGggPiBsZW4pIHtcbiAgICBmaXggPSBudW0udG9GaXhlZChNYXRoLm1heCgwLCBsZW4gLSAoZml4Lmxlbmd0aCAtIGxlbikpKTtcbiAgfVxuICBpZiAoXG4gICAgZml4Lmxlbmd0aCA8PSBsZW4gJiZcbiAgICAhZml4LnN0YXJ0c1dpdGgoJzAuMDAwJykgJiZcbiAgICAhZml4LnN0YXJ0c1dpdGgoJy0wLjAwMCcpXG4gICkge1xuICAgIHJldHVybiBmaXg7XG4gIH1cblxuICAvLyB3ZWxsLCBpZiBpdCdzIHN0aWxsIHRvbyBsb25nIHRoZSB1c2VyIHNob3VsZCd2ZSB1c2VkIGxvbmdlciBudW1iZXJzXG4gIGxldCBleHAgPSBudW0udG9FeHBvbmVudGlhbChsZW4pO1xuICBpZiAoZXhwLmxlbmd0aCA+IGxlbikge1xuICAgIGV4cCA9IG51bS50b0V4cG9uZW50aWFsKE1hdGgubWF4KDAsIGxlbiAtIChleHAubGVuZ3RoIC0gbGVuKSkpO1xuICB9XG4gIHJldHVybiBleHAuc2xpY2UoMCk7XG59XG4iLCJpbXBvcnQgU2luZ3VsYXJWYWx1ZURlY29tcG9zaXRpb24gZnJvbSAnLi9kYy9zdmQnO1xuaW1wb3J0IE1hdHJpeCBmcm9tICcuL21hdHJpeCc7XG5cbmZ1bmN0aW9uIHhyYW5nZShuLCBleGNlcHRpb24pIHtcbiAgbGV0IHJhbmdlID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbjsgaSsrKSB7XG4gICAgaWYgKGkgIT09IGV4Y2VwdGlvbikge1xuICAgICAgcmFuZ2UucHVzaChpKTtcbiAgICB9XG4gIH1cbiAgcmV0dXJuIHJhbmdlO1xufVxuXG5mdW5jdGlvbiBkZXBlbmRlbmNpZXNPbmVSb3coXG4gIGVycm9yLFxuICBtYXRyaXgsXG4gIGluZGV4LFxuICB0aHJlc2hvbGRWYWx1ZSA9IDEwZS0xMCxcbiAgdGhyZXNob2xkRXJyb3IgPSAxMGUtMTAsXG4pIHtcbiAgaWYgKGVycm9yID4gdGhyZXNob2xkRXJyb3IpIHtcbiAgICByZXR1cm4gbmV3IEFycmF5KG1hdHJpeC5yb3dzICsgMSkuZmlsbCgwKTtcbiAgfSBlbHNlIHtcbiAgICBsZXQgcmV0dXJuQXJyYXkgPSBtYXRyaXguYWRkUm93KGluZGV4LCBbMF0pO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcmV0dXJuQXJyYXkucm93czsgaSsrKSB7XG4gICAgICBpZiAoTWF0aC5hYnMocmV0dXJuQXJyYXkuZ2V0KGksIDApKSA8IHRocmVzaG9sZFZhbHVlKSB7XG4gICAgICAgIHJldHVybkFycmF5LnNldChpLCAwLCAwKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHJldHVybkFycmF5LnRvMURBcnJheSgpO1xuICB9XG59XG5cbmV4cG9ydCBmdW5jdGlvbiBsaW5lYXJEZXBlbmRlbmNpZXMobWF0cml4LCBvcHRpb25zID0ge30pIHtcbiAgY29uc3QgeyB0aHJlc2hvbGRWYWx1ZSA9IDEwZS0xMCwgdGhyZXNob2xkRXJyb3IgPSAxMGUtMTAgfSA9IG9wdGlvbnM7XG4gIG1hdHJpeCA9IE1hdHJpeC5jaGVja01hdHJpeChtYXRyaXgpO1xuXG4gIGxldCBuID0gbWF0cml4LnJvd3M7XG4gIGxldCByZXN1bHRzID0gbmV3IE1hdHJpeChuLCBuKTtcblxuICBmb3IgKGxldCBpID0gMDsgaSA8IG47IGkrKykge1xuICAgIGxldCBiID0gTWF0cml4LmNvbHVtblZlY3RvcihtYXRyaXguZ2V0Um93KGkpKTtcbiAgICBsZXQgQWJpcyA9IG1hdHJpeC5zdWJNYXRyaXhSb3coeHJhbmdlKG4sIGkpKS50cmFuc3Bvc2UoKTtcbiAgICBsZXQgc3ZkID0gbmV3IFNpbmd1bGFyVmFsdWVEZWNvbXBvc2l0aW9uKEFiaXMpO1xuICAgIGxldCB4ID0gc3ZkLnNvbHZlKGIpO1xuICAgIGxldCBlcnJvciA9IE1hdHJpeC5zdWIoYiwgQWJpcy5tbXVsKHgpKS5hYnMoKS5tYXgoKTtcbiAgICByZXN1bHRzLnNldFJvdyhcbiAgICAgIGksXG4gICAgICBkZXBlbmRlbmNpZXNPbmVSb3coZXJyb3IsIHgsIGksIHRocmVzaG9sZFZhbHVlLCB0aHJlc2hvbGRFcnJvciksXG4gICAgKTtcbiAgfVxuICByZXR1cm4gcmVzdWx0cztcbn1cbiIsImV4cG9ydCBmdW5jdGlvbiBpbnN0YWxsTWF0aE9wZXJhdGlvbnMoQWJzdHJhY3RNYXRyaXgsIE1hdHJpeCkge1xuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuYWRkID0gZnVuY3Rpb24gYWRkKHZhbHVlKSB7XG4gICAgaWYgKHR5cGVvZiB2YWx1ZSA9PT0gJ251bWJlcicpIHJldHVybiB0aGlzLmFkZFModmFsdWUpO1xuICAgIHJldHVybiB0aGlzLmFkZE0odmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5hZGRTID0gZnVuY3Rpb24gYWRkUyh2YWx1ZSkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgKyB2YWx1ZSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5hZGRNID0gZnVuY3Rpb24gYWRkTShtYXRyaXgpIHtcbiAgICBtYXRyaXggPSBNYXRyaXguY2hlY2tNYXRyaXgobWF0cml4KTtcbiAgICBpZiAodGhpcy5yb3dzICE9PSBtYXRyaXgucm93cyB8fFxuICAgICAgdGhpcy5jb2x1bW5zICE9PSBtYXRyaXguY29sdW1ucykge1xuICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ01hdHJpY2VzIGRpbWVuc2lvbnMgbXVzdCBiZSBlcXVhbCcpO1xuICAgIH1cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopICsgbWF0cml4LmdldChpLCBqKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmFkZCA9IGZ1bmN0aW9uIGFkZChtYXRyaXgsIHZhbHVlKSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguYWRkKHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuc3ViID0gZnVuY3Rpb24gc3ViKHZhbHVlKSB7XG4gICAgaWYgKHR5cGVvZiB2YWx1ZSA9PT0gJ251bWJlcicpIHJldHVybiB0aGlzLnN1YlModmFsdWUpO1xuICAgIHJldHVybiB0aGlzLnN1Yk0odmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zdWJTID0gZnVuY3Rpb24gc3ViUyh2YWx1ZSkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgLSB2YWx1ZSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zdWJNID0gZnVuY3Rpb24gc3ViTShtYXRyaXgpIHtcbiAgICBtYXRyaXggPSBNYXRyaXguY2hlY2tNYXRyaXgobWF0cml4KTtcbiAgICBpZiAodGhpcy5yb3dzICE9PSBtYXRyaXgucm93cyB8fFxuICAgICAgdGhpcy5jb2x1bW5zICE9PSBtYXRyaXguY29sdW1ucykge1xuICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ01hdHJpY2VzIGRpbWVuc2lvbnMgbXVzdCBiZSBlcXVhbCcpO1xuICAgIH1cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIC0gbWF0cml4LmdldChpLCBqKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnN1YiA9IGZ1bmN0aW9uIHN1YihtYXRyaXgsIHZhbHVlKSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguc3ViKHZhbHVlKTtcbiAgfTtcbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnN1YnRyYWN0ID0gQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnN1YjtcbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnN1YnRyYWN0UyA9IEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zdWJTO1xuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuc3VidHJhY3RNID0gQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnN1Yk07XG4gIEFic3RyYWN0TWF0cml4LnN1YnRyYWN0ID0gQWJzdHJhY3RNYXRyaXguc3ViO1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5tdWwgPSBmdW5jdGlvbiBtdWwodmFsdWUpIHtcbiAgICBpZiAodHlwZW9mIHZhbHVlID09PSAnbnVtYmVyJykgcmV0dXJuIHRoaXMubXVsUyh2YWx1ZSk7XG4gICAgcmV0dXJuIHRoaXMubXVsTSh2YWx1ZSk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm11bFMgPSBmdW5jdGlvbiBtdWxTKHZhbHVlKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSAqIHZhbHVlKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm11bE0gPSBmdW5jdGlvbiBtdWxNKG1hdHJpeCkge1xuICAgIG1hdHJpeCA9IE1hdHJpeC5jaGVja01hdHJpeChtYXRyaXgpO1xuICAgIGlmICh0aGlzLnJvd3MgIT09IG1hdHJpeC5yb3dzIHx8XG4gICAgICB0aGlzLmNvbHVtbnMgIT09IG1hdHJpeC5jb2x1bW5zKSB7XG4gICAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcignTWF0cmljZXMgZGltZW5zaW9ucyBtdXN0IGJlIGVxdWFsJyk7XG4gICAgfVxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgKiBtYXRyaXguZ2V0KGksIGopKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgubXVsID0gZnVuY3Rpb24gbXVsKG1hdHJpeCwgdmFsdWUpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5tdWwodmFsdWUpO1xuICB9O1xuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubXVsdGlwbHkgPSBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubXVsO1xuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubXVsdGlwbHlTID0gQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm11bFM7XG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5tdWx0aXBseU0gPSBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubXVsTTtcbiAgQWJzdHJhY3RNYXRyaXgubXVsdGlwbHkgPSBBYnN0cmFjdE1hdHJpeC5tdWw7XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmRpdiA9IGZ1bmN0aW9uIGRpdih2YWx1ZSkge1xuICAgIGlmICh0eXBlb2YgdmFsdWUgPT09ICdudW1iZXInKSByZXR1cm4gdGhpcy5kaXZTKHZhbHVlKTtcbiAgICByZXR1cm4gdGhpcy5kaXZNKHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuZGl2UyA9IGZ1bmN0aW9uIGRpdlModmFsdWUpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIC8gdmFsdWUpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuZGl2TSA9IGZ1bmN0aW9uIGRpdk0obWF0cml4KSB7XG4gICAgbWF0cml4ID0gTWF0cml4LmNoZWNrTWF0cml4KG1hdHJpeCk7XG4gICAgaWYgKHRoaXMucm93cyAhPT0gbWF0cml4LnJvd3MgfHxcbiAgICAgIHRoaXMuY29sdW1ucyAhPT0gbWF0cml4LmNvbHVtbnMpIHtcbiAgICAgIHRocm93IG5ldyBSYW5nZUVycm9yKCdNYXRyaWNlcyBkaW1lbnNpb25zIG11c3QgYmUgZXF1YWwnKTtcbiAgICB9XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSAvIG1hdHJpeC5nZXQoaSwgaikpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5kaXYgPSBmdW5jdGlvbiBkaXYobWF0cml4LCB2YWx1ZSkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LmRpdih2YWx1ZSk7XG4gIH07XG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5kaXZpZGUgPSBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuZGl2O1xuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuZGl2aWRlUyA9IEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5kaXZTO1xuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuZGl2aWRlTSA9IEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5kaXZNO1xuICBBYnN0cmFjdE1hdHJpeC5kaXZpZGUgPSBBYnN0cmFjdE1hdHJpeC5kaXY7XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm1vZCA9IGZ1bmN0aW9uIG1vZCh2YWx1ZSkge1xuICAgIGlmICh0eXBlb2YgdmFsdWUgPT09ICdudW1iZXInKSByZXR1cm4gdGhpcy5tb2RTKHZhbHVlKTtcbiAgICByZXR1cm4gdGhpcy5tb2RNKHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubW9kUyA9IGZ1bmN0aW9uIG1vZFModmFsdWUpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopICUgdmFsdWUpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubW9kTSA9IGZ1bmN0aW9uIG1vZE0obWF0cml4KSB7XG4gICAgbWF0cml4ID0gTWF0cml4LmNoZWNrTWF0cml4KG1hdHJpeCk7XG4gICAgaWYgKHRoaXMucm93cyAhPT0gbWF0cml4LnJvd3MgfHxcbiAgICAgIHRoaXMuY29sdW1ucyAhPT0gbWF0cml4LmNvbHVtbnMpIHtcbiAgICAgIHRocm93IG5ldyBSYW5nZUVycm9yKCdNYXRyaWNlcyBkaW1lbnNpb25zIG11c3QgYmUgZXF1YWwnKTtcbiAgICB9XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSAlIG1hdHJpeC5nZXQoaSwgaikpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5tb2QgPSBmdW5jdGlvbiBtb2QobWF0cml4LCB2YWx1ZSkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4Lm1vZCh2YWx1ZSk7XG4gIH07XG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5tb2R1bHVzID0gQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm1vZDtcbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm1vZHVsdXNTID0gQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm1vZFM7XG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5tb2R1bHVzTSA9IEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5tb2RNO1xuICBBYnN0cmFjdE1hdHJpeC5tb2R1bHVzID0gQWJzdHJhY3RNYXRyaXgubW9kO1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5hbmQgPSBmdW5jdGlvbiBhbmQodmFsdWUpIHtcbiAgICBpZiAodHlwZW9mIHZhbHVlID09PSAnbnVtYmVyJykgcmV0dXJuIHRoaXMuYW5kUyh2YWx1ZSk7XG4gICAgcmV0dXJuIHRoaXMuYW5kTSh2YWx1ZSk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmFuZFMgPSBmdW5jdGlvbiBhbmRTKHZhbHVlKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSAmIHZhbHVlKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmFuZE0gPSBmdW5jdGlvbiBhbmRNKG1hdHJpeCkge1xuICAgIG1hdHJpeCA9IE1hdHJpeC5jaGVja01hdHJpeChtYXRyaXgpO1xuICAgIGlmICh0aGlzLnJvd3MgIT09IG1hdHJpeC5yb3dzIHx8XG4gICAgICB0aGlzLmNvbHVtbnMgIT09IG1hdHJpeC5jb2x1bW5zKSB7XG4gICAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcignTWF0cmljZXMgZGltZW5zaW9ucyBtdXN0IGJlIGVxdWFsJyk7XG4gICAgfVxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgJiBtYXRyaXguZ2V0KGksIGopKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXguYW5kID0gZnVuY3Rpb24gYW5kKG1hdHJpeCwgdmFsdWUpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5hbmQodmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5vciA9IGZ1bmN0aW9uIG9yKHZhbHVlKSB7XG4gICAgaWYgKHR5cGVvZiB2YWx1ZSA9PT0gJ251bWJlcicpIHJldHVybiB0aGlzLm9yUyh2YWx1ZSk7XG4gICAgcmV0dXJuIHRoaXMub3JNKHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUub3JTID0gZnVuY3Rpb24gb3JTKHZhbHVlKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSB8IHZhbHVlKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm9yTSA9IGZ1bmN0aW9uIG9yTShtYXRyaXgpIHtcbiAgICBtYXRyaXggPSBNYXRyaXguY2hlY2tNYXRyaXgobWF0cml4KTtcbiAgICBpZiAodGhpcy5yb3dzICE9PSBtYXRyaXgucm93cyB8fFxuICAgICAgdGhpcy5jb2x1bW5zICE9PSBtYXRyaXguY29sdW1ucykge1xuICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ01hdHJpY2VzIGRpbWVuc2lvbnMgbXVzdCBiZSBlcXVhbCcpO1xuICAgIH1cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIHwgbWF0cml4LmdldChpLCBqKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4Lm9yID0gZnVuY3Rpb24gb3IobWF0cml4LCB2YWx1ZSkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4Lm9yKHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUueG9yID0gZnVuY3Rpb24geG9yKHZhbHVlKSB7XG4gICAgaWYgKHR5cGVvZiB2YWx1ZSA9PT0gJ251bWJlcicpIHJldHVybiB0aGlzLnhvclModmFsdWUpO1xuICAgIHJldHVybiB0aGlzLnhvck0odmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS54b3JTID0gZnVuY3Rpb24geG9yUyh2YWx1ZSkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgXiB2YWx1ZSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS54b3JNID0gZnVuY3Rpb24geG9yTShtYXRyaXgpIHtcbiAgICBtYXRyaXggPSBNYXRyaXguY2hlY2tNYXRyaXgobWF0cml4KTtcbiAgICBpZiAodGhpcy5yb3dzICE9PSBtYXRyaXgucm93cyB8fFxuICAgICAgdGhpcy5jb2x1bW5zICE9PSBtYXRyaXguY29sdW1ucykge1xuICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ01hdHJpY2VzIGRpbWVuc2lvbnMgbXVzdCBiZSBlcXVhbCcpO1xuICAgIH1cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIF4gbWF0cml4LmdldChpLCBqKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnhvciA9IGZ1bmN0aW9uIHhvcihtYXRyaXgsIHZhbHVlKSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXgueG9yKHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubGVmdFNoaWZ0ID0gZnVuY3Rpb24gbGVmdFNoaWZ0KHZhbHVlKSB7XG4gICAgaWYgKHR5cGVvZiB2YWx1ZSA9PT0gJ251bWJlcicpIHJldHVybiB0aGlzLmxlZnRTaGlmdFModmFsdWUpO1xuICAgIHJldHVybiB0aGlzLmxlZnRTaGlmdE0odmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5sZWZ0U2hpZnRTID0gZnVuY3Rpb24gbGVmdFNoaWZ0Uyh2YWx1ZSkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgPDwgdmFsdWUpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubGVmdFNoaWZ0TSA9IGZ1bmN0aW9uIGxlZnRTaGlmdE0obWF0cml4KSB7XG4gICAgbWF0cml4ID0gTWF0cml4LmNoZWNrTWF0cml4KG1hdHJpeCk7XG4gICAgaWYgKHRoaXMucm93cyAhPT0gbWF0cml4LnJvd3MgfHxcbiAgICAgIHRoaXMuY29sdW1ucyAhPT0gbWF0cml4LmNvbHVtbnMpIHtcbiAgICAgIHRocm93IG5ldyBSYW5nZUVycm9yKCdNYXRyaWNlcyBkaW1lbnNpb25zIG11c3QgYmUgZXF1YWwnKTtcbiAgICB9XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSA8PCBtYXRyaXguZ2V0KGksIGopKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgubGVmdFNoaWZ0ID0gZnVuY3Rpb24gbGVmdFNoaWZ0KG1hdHJpeCwgdmFsdWUpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5sZWZ0U2hpZnQodmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zaWduUHJvcGFnYXRpbmdSaWdodFNoaWZ0ID0gZnVuY3Rpb24gc2lnblByb3BhZ2F0aW5nUmlnaHRTaGlmdCh2YWx1ZSkge1xuICAgIGlmICh0eXBlb2YgdmFsdWUgPT09ICdudW1iZXInKSByZXR1cm4gdGhpcy5zaWduUHJvcGFnYXRpbmdSaWdodFNoaWZ0Uyh2YWx1ZSk7XG4gICAgcmV0dXJuIHRoaXMuc2lnblByb3BhZ2F0aW5nUmlnaHRTaGlmdE0odmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zaWduUHJvcGFnYXRpbmdSaWdodFNoaWZ0UyA9IGZ1bmN0aW9uIHNpZ25Qcm9wYWdhdGluZ1JpZ2h0U2hpZnRTKHZhbHVlKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSA+PiB2YWx1ZSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zaWduUHJvcGFnYXRpbmdSaWdodFNoaWZ0TSA9IGZ1bmN0aW9uIHNpZ25Qcm9wYWdhdGluZ1JpZ2h0U2hpZnRNKG1hdHJpeCkge1xuICAgIG1hdHJpeCA9IE1hdHJpeC5jaGVja01hdHJpeChtYXRyaXgpO1xuICAgIGlmICh0aGlzLnJvd3MgIT09IG1hdHJpeC5yb3dzIHx8XG4gICAgICB0aGlzLmNvbHVtbnMgIT09IG1hdHJpeC5jb2x1bW5zKSB7XG4gICAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcignTWF0cmljZXMgZGltZW5zaW9ucyBtdXN0IGJlIGVxdWFsJyk7XG4gICAgfVxuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgPj4gbWF0cml4LmdldChpLCBqKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnNpZ25Qcm9wYWdhdGluZ1JpZ2h0U2hpZnQgPSBmdW5jdGlvbiBzaWduUHJvcGFnYXRpbmdSaWdodFNoaWZ0KG1hdHJpeCwgdmFsdWUpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5zaWduUHJvcGFnYXRpbmdSaWdodFNoaWZ0KHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUucmlnaHRTaGlmdCA9IGZ1bmN0aW9uIHJpZ2h0U2hpZnQodmFsdWUpIHtcbiAgICBpZiAodHlwZW9mIHZhbHVlID09PSAnbnVtYmVyJykgcmV0dXJuIHRoaXMucmlnaHRTaGlmdFModmFsdWUpO1xuICAgIHJldHVybiB0aGlzLnJpZ2h0U2hpZnRNKHZhbHVlKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUucmlnaHRTaGlmdFMgPSBmdW5jdGlvbiByaWdodFNoaWZ0Uyh2YWx1ZSkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgdGhpcy5nZXQoaSwgaikgPj4+IHZhbHVlKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnJpZ2h0U2hpZnRNID0gZnVuY3Rpb24gcmlnaHRTaGlmdE0obWF0cml4KSB7XG4gICAgbWF0cml4ID0gTWF0cml4LmNoZWNrTWF0cml4KG1hdHJpeCk7XG4gICAgaWYgKHRoaXMucm93cyAhPT0gbWF0cml4LnJvd3MgfHxcbiAgICAgIHRoaXMuY29sdW1ucyAhPT0gbWF0cml4LmNvbHVtbnMpIHtcbiAgICAgIHRocm93IG5ldyBSYW5nZUVycm9yKCdNYXRyaWNlcyBkaW1lbnNpb25zIG11c3QgYmUgZXF1YWwnKTtcbiAgICB9XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCB0aGlzLmdldChpLCBqKSA+Pj4gbWF0cml4LmdldChpLCBqKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnJpZ2h0U2hpZnQgPSBmdW5jdGlvbiByaWdodFNoaWZ0KG1hdHJpeCwgdmFsdWUpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5yaWdodFNoaWZ0KHZhbHVlKTtcbiAgfTtcbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnplcm9GaWxsUmlnaHRTaGlmdCA9IEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5yaWdodFNoaWZ0O1xuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuemVyb0ZpbGxSaWdodFNoaWZ0UyA9IEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5yaWdodFNoaWZ0UztcbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnplcm9GaWxsUmlnaHRTaGlmdE0gPSBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUucmlnaHRTaGlmdE07XG4gIEFic3RyYWN0TWF0cml4Lnplcm9GaWxsUmlnaHRTaGlmdCA9IEFic3RyYWN0TWF0cml4LnJpZ2h0U2hpZnQ7XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm5vdCA9IGZ1bmN0aW9uIG5vdCgpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIH4odGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgubm90ID0gZnVuY3Rpb24gbm90KG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4Lm5vdCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5hYnMgPSBmdW5jdGlvbiBhYnMoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmFicyh0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5hYnMgPSBmdW5jdGlvbiBhYnMobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguYWJzKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmFjb3MgPSBmdW5jdGlvbiBhY29zKCkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC5hY29zKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmFjb3MgPSBmdW5jdGlvbiBhY29zKG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LmFjb3MoKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuYWNvc2ggPSBmdW5jdGlvbiBhY29zaCgpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGguYWNvc2godGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXguYWNvc2ggPSBmdW5jdGlvbiBhY29zaChtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5hY29zaCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5hc2luID0gZnVuY3Rpb24gYXNpbigpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGguYXNpbih0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5hc2luID0gZnVuY3Rpb24gYXNpbihtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5hc2luKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmFzaW5oID0gZnVuY3Rpb24gYXNpbmgoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmFzaW5oKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmFzaW5oID0gZnVuY3Rpb24gYXNpbmgobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguYXNpbmgoKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuYXRhbiA9IGZ1bmN0aW9uIGF0YW4oKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmF0YW4odGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXguYXRhbiA9IGZ1bmN0aW9uIGF0YW4obWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguYXRhbigpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5hdGFuaCA9IGZ1bmN0aW9uIGF0YW5oKCkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC5hdGFuaCh0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5hdGFuaCA9IGZ1bmN0aW9uIGF0YW5oKG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LmF0YW5oKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmNicnQgPSBmdW5jdGlvbiBjYnJ0KCkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC5jYnJ0KHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmNicnQgPSBmdW5jdGlvbiBjYnJ0KG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LmNicnQoKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuY2VpbCA9IGZ1bmN0aW9uIGNlaWwoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmNlaWwodGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXguY2VpbCA9IGZ1bmN0aW9uIGNlaWwobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguY2VpbCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5jbHozMiA9IGZ1bmN0aW9uIGNsejMyKCkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC5jbHozMih0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5jbHozMiA9IGZ1bmN0aW9uIGNsejMyKG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LmNsejMyKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmNvcyA9IGZ1bmN0aW9uIGNvcygpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGguY29zKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmNvcyA9IGZ1bmN0aW9uIGNvcyhtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5jb3MoKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuY29zaCA9IGZ1bmN0aW9uIGNvc2goKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmNvc2godGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXguY29zaCA9IGZ1bmN0aW9uIGNvc2gobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguY29zaCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5leHAgPSBmdW5jdGlvbiBleHAoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmV4cCh0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5leHAgPSBmdW5jdGlvbiBleHAobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguZXhwKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmV4cG0xID0gZnVuY3Rpb24gZXhwbTEoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmV4cG0xKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmV4cG0xID0gZnVuY3Rpb24gZXhwbTEobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguZXhwbTEoKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuZmxvb3IgPSBmdW5jdGlvbiBmbG9vcigpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGguZmxvb3IodGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXguZmxvb3IgPSBmdW5jdGlvbiBmbG9vcihtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5mbG9vcigpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5mcm91bmQgPSBmdW5jdGlvbiBmcm91bmQoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLmZyb3VuZCh0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5mcm91bmQgPSBmdW5jdGlvbiBmcm91bmQobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguZnJvdW5kKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmxvZyA9IGZ1bmN0aW9uIGxvZygpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGgubG9nKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmxvZyA9IGZ1bmN0aW9uIGxvZyhtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5sb2coKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubG9nMXAgPSBmdW5jdGlvbiBsb2cxcCgpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGgubG9nMXAodGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgubG9nMXAgPSBmdW5jdGlvbiBsb2cxcChtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5sb2cxcCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5sb2cxMCA9IGZ1bmN0aW9uIGxvZzEwKCkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC5sb2cxMCh0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5sb2cxMCA9IGZ1bmN0aW9uIGxvZzEwKG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LmxvZzEwKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmxvZzIgPSBmdW5jdGlvbiBsb2cyKCkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC5sb2cyKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LmxvZzIgPSBmdW5jdGlvbiBsb2cyKG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LmxvZzIoKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUucm91bmQgPSBmdW5jdGlvbiByb3VuZCgpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGgucm91bmQodGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucm91bmQgPSBmdW5jdGlvbiByb3VuZChtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5yb3VuZCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zaWduID0gZnVuY3Rpb24gc2lnbigpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGguc2lnbih0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5zaWduID0gZnVuY3Rpb24gc2lnbihtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5zaWduKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnNpbiA9IGZ1bmN0aW9uIHNpbigpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGguc2luKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnNpbiA9IGZ1bmN0aW9uIHNpbihtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5zaW4oKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUuc2luaCA9IGZ1bmN0aW9uIHNpbmgoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLnNpbmgodGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXguc2luaCA9IGZ1bmN0aW9uIHNpbmgobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXguc2luaCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5zcXJ0ID0gZnVuY3Rpb24gc3FydCgpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGguc3FydCh0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5zcXJ0ID0gZnVuY3Rpb24gc3FydChtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC5zcXJ0KCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLnRhbiA9IGZ1bmN0aW9uIHRhbigpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGgudGFuKHRoaXMuZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnRhbiA9IGZ1bmN0aW9uIHRhbihtYXRyaXgpIHtcbiAgICBjb25zdCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KG1hdHJpeCk7XG4gICAgcmV0dXJuIG5ld01hdHJpeC50YW4oKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUudGFuaCA9IGZ1bmN0aW9uIHRhbmgoKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICB0aGlzLnNldChpLCBqLCBNYXRoLnRhbmgodGhpcy5nZXQoaSwgaikpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgudGFuaCA9IGZ1bmN0aW9uIHRhbmgobWF0cml4KSB7XG4gICAgY29uc3QgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChtYXRyaXgpO1xuICAgIHJldHVybiBuZXdNYXRyaXgudGFuaCgpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS50cnVuYyA9IGZ1bmN0aW9uIHRydW5jKCkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC50cnVuYyh0aGlzLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC50cnVuYyA9IGZ1bmN0aW9uIHRydW5jKG1hdHJpeCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LnRydW5jKCk7XG4gIH07XG5cbiAgQWJzdHJhY3RNYXRyaXgucG93ID0gZnVuY3Rpb24gcG93KG1hdHJpeCwgYXJnMCkge1xuICAgIGNvbnN0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgobWF0cml4KTtcbiAgICByZXR1cm4gbmV3TWF0cml4LnBvdyhhcmcwKTtcbiAgfTtcblxuICBBYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUucG93ID0gZnVuY3Rpb24gcG93KHZhbHVlKSB7XG4gICAgaWYgKHR5cGVvZiB2YWx1ZSA9PT0gJ251bWJlcicpIHJldHVybiB0aGlzLnBvd1ModmFsdWUpO1xuICAgIHJldHVybiB0aGlzLnBvd00odmFsdWUpO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5wb3dTID0gZnVuY3Rpb24gcG93Uyh2YWx1ZSkge1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoaSwgaiwgTWF0aC5wb3codGhpcy5nZXQoaSwgaiksIHZhbHVlKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xuXG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5wb3dNID0gZnVuY3Rpb24gcG93TShtYXRyaXgpIHtcbiAgICBtYXRyaXggPSBNYXRyaXguY2hlY2tNYXRyaXgobWF0cml4KTtcbiAgICBpZiAodGhpcy5yb3dzICE9PSBtYXRyaXgucm93cyB8fFxuICAgICAgdGhpcy5jb2x1bW5zICE9PSBtYXRyaXguY29sdW1ucykge1xuICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ01hdHJpY2VzIGRpbWVuc2lvbnMgbXVzdCBiZSBlcXVhbCcpO1xuICAgIH1cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIE1hdGgucG93KHRoaXMuZ2V0KGksIGopLCBtYXRyaXguZ2V0KGksIGopKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9O1xufVxuIiwiaW1wb3J0IHsgaXNBbnlBcnJheSB9IGZyb20gJ2lzLWFueS1hcnJheSc7XG5pbXBvcnQgcmVzY2FsZSBmcm9tICdtbC1hcnJheS1yZXNjYWxlJztcblxuaW1wb3J0IHsgaW5zcGVjdE1hdHJpeCwgaW5zcGVjdE1hdHJpeFdpdGhPcHRpb25zIH0gZnJvbSAnLi9pbnNwZWN0JztcbmltcG9ydCB7IGluc3RhbGxNYXRoT3BlcmF0aW9ucyB9IGZyb20gJy4vbWF0aE9wZXJhdGlvbnMnO1xuaW1wb3J0IHtcbiAgc3VtQnlSb3csXG4gIHN1bUJ5Q29sdW1uLFxuICBzdW1BbGwsXG4gIHByb2R1Y3RCeVJvdyxcbiAgcHJvZHVjdEJ5Q29sdW1uLFxuICBwcm9kdWN0QWxsLFxuICB2YXJpYW5jZUJ5Um93LFxuICB2YXJpYW5jZUJ5Q29sdW1uLFxuICB2YXJpYW5jZUFsbCxcbiAgY2VudGVyQnlSb3csXG4gIGNlbnRlckJ5Q29sdW1uLFxuICBjZW50ZXJBbGwsXG4gIHNjYWxlQnlSb3csXG4gIHNjYWxlQnlDb2x1bW4sXG4gIHNjYWxlQWxsLFxuICBnZXRTY2FsZUJ5Um93LFxuICBnZXRTY2FsZUJ5Q29sdW1uLFxuICBnZXRTY2FsZUFsbCxcbn0gZnJvbSAnLi9zdGF0JztcbmltcG9ydCB7XG4gIGNoZWNrUm93VmVjdG9yLFxuICBjaGVja1Jvd0luZGV4LFxuICBjaGVja0NvbHVtbkluZGV4LFxuICBjaGVja0NvbHVtblZlY3RvcixcbiAgY2hlY2tSYW5nZSxcbiAgY2hlY2tOb25FbXB0eSxcbiAgY2hlY2tSb3dJbmRpY2VzLFxuICBjaGVja0NvbHVtbkluZGljZXMsXG59IGZyb20gJy4vdXRpbCc7XG5cbmV4cG9ydCBjbGFzcyBBYnN0cmFjdE1hdHJpeCB7XG4gIHN0YXRpYyBmcm9tMURBcnJheShuZXdSb3dzLCBuZXdDb2x1bW5zLCBuZXdEYXRhKSB7XG4gICAgbGV0IGxlbmd0aCA9IG5ld1Jvd3MgKiBuZXdDb2x1bW5zO1xuICAgIGlmIChsZW5ndGggIT09IG5ld0RhdGEubGVuZ3RoKSB7XG4gICAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcignZGF0YSBsZW5ndGggZG9lcyBub3QgbWF0Y2ggZ2l2ZW4gZGltZW5zaW9ucycpO1xuICAgIH1cbiAgICBsZXQgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChuZXdSb3dzLCBuZXdDb2x1bW5zKTtcbiAgICBmb3IgKGxldCByb3cgPSAwOyByb3cgPCBuZXdSb3dzOyByb3crKykge1xuICAgICAgZm9yIChsZXQgY29sdW1uID0gMDsgY29sdW1uIDwgbmV3Q29sdW1uczsgY29sdW1uKyspIHtcbiAgICAgICAgbmV3TWF0cml4LnNldChyb3csIGNvbHVtbiwgbmV3RGF0YVtyb3cgKiBuZXdDb2x1bW5zICsgY29sdW1uXSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBuZXdNYXRyaXg7XG4gIH1cblxuICBzdGF0aWMgcm93VmVjdG9yKG5ld0RhdGEpIHtcbiAgICBsZXQgdmVjdG9yID0gbmV3IE1hdHJpeCgxLCBuZXdEYXRhLmxlbmd0aCk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBuZXdEYXRhLmxlbmd0aDsgaSsrKSB7XG4gICAgICB2ZWN0b3Iuc2V0KDAsIGksIG5ld0RhdGFbaV0pO1xuICAgIH1cbiAgICByZXR1cm4gdmVjdG9yO1xuICB9XG5cbiAgc3RhdGljIGNvbHVtblZlY3RvcihuZXdEYXRhKSB7XG4gICAgbGV0IHZlY3RvciA9IG5ldyBNYXRyaXgobmV3RGF0YS5sZW5ndGgsIDEpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbmV3RGF0YS5sZW5ndGg7IGkrKykge1xuICAgICAgdmVjdG9yLnNldChpLCAwLCBuZXdEYXRhW2ldKTtcbiAgICB9XG4gICAgcmV0dXJuIHZlY3RvcjtcbiAgfVxuXG4gIHN0YXRpYyB6ZXJvcyhyb3dzLCBjb2x1bW5zKSB7XG4gICAgcmV0dXJuIG5ldyBNYXRyaXgocm93cywgY29sdW1ucyk7XG4gIH1cblxuICBzdGF0aWMgb25lcyhyb3dzLCBjb2x1bW5zKSB7XG4gICAgcmV0dXJuIG5ldyBNYXRyaXgocm93cywgY29sdW1ucykuZmlsbCgxKTtcbiAgfVxuXG4gIHN0YXRpYyByYW5kKHJvd3MsIGNvbHVtbnMsIG9wdGlvbnMgPSB7fSkge1xuICAgIGlmICh0eXBlb2Ygb3B0aW9ucyAhPT0gJ29iamVjdCcpIHtcbiAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ29wdGlvbnMgbXVzdCBiZSBhbiBvYmplY3QnKTtcbiAgICB9XG4gICAgY29uc3QgeyByYW5kb20gPSBNYXRoLnJhbmRvbSB9ID0gb3B0aW9ucztcbiAgICBsZXQgbWF0cml4ID0gbmV3IE1hdHJpeChyb3dzLCBjb2x1bW5zKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBjb2x1bW5zOyBqKyspIHtcbiAgICAgICAgbWF0cml4LnNldChpLCBqLCByYW5kb20oKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBtYXRyaXg7XG4gIH1cblxuICBzdGF0aWMgcmFuZEludChyb3dzLCBjb2x1bW5zLCBvcHRpb25zID0ge30pIHtcbiAgICBpZiAodHlwZW9mIG9wdGlvbnMgIT09ICdvYmplY3QnKSB7XG4gICAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdvcHRpb25zIG11c3QgYmUgYW4gb2JqZWN0Jyk7XG4gICAgfVxuICAgIGNvbnN0IHsgbWluID0gMCwgbWF4ID0gMTAwMCwgcmFuZG9tID0gTWF0aC5yYW5kb20gfSA9IG9wdGlvbnM7XG4gICAgaWYgKCFOdW1iZXIuaXNJbnRlZ2VyKG1pbikpIHRocm93IG5ldyBUeXBlRXJyb3IoJ21pbiBtdXN0IGJlIGFuIGludGVnZXInKTtcbiAgICBpZiAoIU51bWJlci5pc0ludGVnZXIobWF4KSkgdGhyb3cgbmV3IFR5cGVFcnJvcignbWF4IG11c3QgYmUgYW4gaW50ZWdlcicpO1xuICAgIGlmIChtaW4gPj0gbWF4KSB0aHJvdyBuZXcgUmFuZ2VFcnJvcignbWluIG11c3QgYmUgc21hbGxlciB0aGFuIG1heCcpO1xuICAgIGxldCBpbnRlcnZhbCA9IG1heCAtIG1pbjtcbiAgICBsZXQgbWF0cml4ID0gbmV3IE1hdHJpeChyb3dzLCBjb2x1bW5zKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBjb2x1bW5zOyBqKyspIHtcbiAgICAgICAgbGV0IHZhbHVlID0gbWluICsgTWF0aC5yb3VuZChyYW5kb20oKSAqIGludGVydmFsKTtcbiAgICAgICAgbWF0cml4LnNldChpLCBqLCB2YWx1ZSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBtYXRyaXg7XG4gIH1cblxuICBzdGF0aWMgZXllKHJvd3MsIGNvbHVtbnMsIHZhbHVlKSB7XG4gICAgaWYgKGNvbHVtbnMgPT09IHVuZGVmaW5lZCkgY29sdW1ucyA9IHJvd3M7XG4gICAgaWYgKHZhbHVlID09PSB1bmRlZmluZWQpIHZhbHVlID0gMTtcbiAgICBsZXQgbWluID0gTWF0aC5taW4ocm93cywgY29sdW1ucyk7XG4gICAgbGV0IG1hdHJpeCA9IHRoaXMuemVyb3Mocm93cywgY29sdW1ucyk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBtaW47IGkrKykge1xuICAgICAgbWF0cml4LnNldChpLCBpLCB2YWx1ZSk7XG4gICAgfVxuICAgIHJldHVybiBtYXRyaXg7XG4gIH1cblxuICBzdGF0aWMgZGlhZyhkYXRhLCByb3dzLCBjb2x1bW5zKSB7XG4gICAgbGV0IGwgPSBkYXRhLmxlbmd0aDtcbiAgICBpZiAocm93cyA9PT0gdW5kZWZpbmVkKSByb3dzID0gbDtcbiAgICBpZiAoY29sdW1ucyA9PT0gdW5kZWZpbmVkKSBjb2x1bW5zID0gcm93cztcbiAgICBsZXQgbWluID0gTWF0aC5taW4obCwgcm93cywgY29sdW1ucyk7XG4gICAgbGV0IG1hdHJpeCA9IHRoaXMuemVyb3Mocm93cywgY29sdW1ucyk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBtaW47IGkrKykge1xuICAgICAgbWF0cml4LnNldChpLCBpLCBkYXRhW2ldKTtcbiAgICB9XG4gICAgcmV0dXJuIG1hdHJpeDtcbiAgfVxuXG4gIHN0YXRpYyBtaW4obWF0cml4MSwgbWF0cml4Mikge1xuICAgIG1hdHJpeDEgPSB0aGlzLmNoZWNrTWF0cml4KG1hdHJpeDEpO1xuICAgIG1hdHJpeDIgPSB0aGlzLmNoZWNrTWF0cml4KG1hdHJpeDIpO1xuICAgIGxldCByb3dzID0gbWF0cml4MS5yb3dzO1xuICAgIGxldCBjb2x1bW5zID0gbWF0cml4MS5jb2x1bW5zO1xuICAgIGxldCByZXN1bHQgPSBuZXcgTWF0cml4KHJvd3MsIGNvbHVtbnMpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IGNvbHVtbnM7IGorKykge1xuICAgICAgICByZXN1bHQuc2V0KGksIGosIE1hdGgubWluKG1hdHJpeDEuZ2V0KGksIGopLCBtYXRyaXgyLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gcmVzdWx0O1xuICB9XG5cbiAgc3RhdGljIG1heChtYXRyaXgxLCBtYXRyaXgyKSB7XG4gICAgbWF0cml4MSA9IHRoaXMuY2hlY2tNYXRyaXgobWF0cml4MSk7XG4gICAgbWF0cml4MiA9IHRoaXMuY2hlY2tNYXRyaXgobWF0cml4Mik7XG4gICAgbGV0IHJvd3MgPSBtYXRyaXgxLnJvd3M7XG4gICAgbGV0IGNvbHVtbnMgPSBtYXRyaXgxLmNvbHVtbnM7XG4gICAgbGV0IHJlc3VsdCA9IG5ldyB0aGlzKHJvd3MsIGNvbHVtbnMpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IGNvbHVtbnM7IGorKykge1xuICAgICAgICByZXN1bHQuc2V0KGksIGosIE1hdGgubWF4KG1hdHJpeDEuZ2V0KGksIGopLCBtYXRyaXgyLmdldChpLCBqKSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gcmVzdWx0O1xuICB9XG5cbiAgc3RhdGljIGNoZWNrTWF0cml4KHZhbHVlKSB7XG4gICAgcmV0dXJuIEFic3RyYWN0TWF0cml4LmlzTWF0cml4KHZhbHVlKSA/IHZhbHVlIDogbmV3IE1hdHJpeCh2YWx1ZSk7XG4gIH1cblxuICBzdGF0aWMgaXNNYXRyaXgodmFsdWUpIHtcbiAgICByZXR1cm4gdmFsdWUgIT0gbnVsbCAmJiB2YWx1ZS5rbGFzcyA9PT0gJ01hdHJpeCc7XG4gIH1cblxuICBnZXQgc2l6ZSgpIHtcbiAgICByZXR1cm4gdGhpcy5yb3dzICogdGhpcy5jb2x1bW5zO1xuICB9XG5cbiAgYXBwbHkoY2FsbGJhY2spIHtcbiAgICBpZiAodHlwZW9mIGNhbGxiYWNrICE9PSAnZnVuY3Rpb24nKSB7XG4gICAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdjYWxsYmFjayBtdXN0IGJlIGEgZnVuY3Rpb24nKTtcbiAgICB9XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICBjYWxsYmFjay5jYWxsKHRoaXMsIGksIGopO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIHRvMURBcnJheSgpIHtcbiAgICBsZXQgYXJyYXkgPSBbXTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIGFycmF5LnB1c2godGhpcy5nZXQoaSwgaikpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gYXJyYXk7XG4gIH1cblxuICB0bzJEQXJyYXkoKSB7XG4gICAgbGV0IGNvcHkgPSBbXTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBjb3B5LnB1c2goW10pO1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICBjb3B5W2ldLnB1c2godGhpcy5nZXQoaSwgaikpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gY29weTtcbiAgfVxuXG4gIHRvSlNPTigpIHtcbiAgICByZXR1cm4gdGhpcy50bzJEQXJyYXkoKTtcbiAgfVxuXG4gIGlzUm93VmVjdG9yKCkge1xuICAgIHJldHVybiB0aGlzLnJvd3MgPT09IDE7XG4gIH1cblxuICBpc0NvbHVtblZlY3RvcigpIHtcbiAgICByZXR1cm4gdGhpcy5jb2x1bW5zID09PSAxO1xuICB9XG5cbiAgaXNWZWN0b3IoKSB7XG4gICAgcmV0dXJuIHRoaXMucm93cyA9PT0gMSB8fCB0aGlzLmNvbHVtbnMgPT09IDE7XG4gIH1cblxuICBpc1NxdWFyZSgpIHtcbiAgICByZXR1cm4gdGhpcy5yb3dzID09PSB0aGlzLmNvbHVtbnM7XG4gIH1cblxuICBpc0VtcHR5KCkge1xuICAgIHJldHVybiB0aGlzLnJvd3MgPT09IDAgfHwgdGhpcy5jb2x1bW5zID09PSAwO1xuICB9XG5cbiAgaXNTeW1tZXRyaWMoKSB7XG4gICAgaWYgKHRoaXMuaXNTcXVhcmUoKSkge1xuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgICBmb3IgKGxldCBqID0gMDsgaiA8PSBpOyBqKyspIHtcbiAgICAgICAgICBpZiAodGhpcy5nZXQoaSwgaikgIT09IHRoaXMuZ2V0KGosIGkpKSB7XG4gICAgICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9XG4gICAgICByZXR1cm4gdHJ1ZTtcbiAgICB9XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgaXNFY2hlbG9uRm9ybSgpIHtcbiAgICBsZXQgaSA9IDA7XG4gICAgbGV0IGogPSAwO1xuICAgIGxldCBwcmV2aW91c0NvbHVtbiA9IC0xO1xuICAgIGxldCBpc0VjaGVsb25Gb3JtID0gdHJ1ZTtcbiAgICBsZXQgY2hlY2tlZCA9IGZhbHNlO1xuICAgIHdoaWxlIChpIDwgdGhpcy5yb3dzICYmIGlzRWNoZWxvbkZvcm0pIHtcbiAgICAgIGogPSAwO1xuICAgICAgY2hlY2tlZCA9IGZhbHNlO1xuICAgICAgd2hpbGUgKGogPCB0aGlzLmNvbHVtbnMgJiYgY2hlY2tlZCA9PT0gZmFsc2UpIHtcbiAgICAgICAgaWYgKHRoaXMuZ2V0KGksIGopID09PSAwKSB7XG4gICAgICAgICAgaisrO1xuICAgICAgICB9IGVsc2UgaWYgKHRoaXMuZ2V0KGksIGopID09PSAxICYmIGogPiBwcmV2aW91c0NvbHVtbikge1xuICAgICAgICAgIGNoZWNrZWQgPSB0cnVlO1xuICAgICAgICAgIHByZXZpb3VzQ29sdW1uID0gajtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICBpc0VjaGVsb25Gb3JtID0gZmFsc2U7XG4gICAgICAgICAgY2hlY2tlZCA9IHRydWU7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIGkrKztcbiAgICB9XG4gICAgcmV0dXJuIGlzRWNoZWxvbkZvcm07XG4gIH1cblxuICBpc1JlZHVjZWRFY2hlbG9uRm9ybSgpIHtcbiAgICBsZXQgaSA9IDA7XG4gICAgbGV0IGogPSAwO1xuICAgIGxldCBwcmV2aW91c0NvbHVtbiA9IC0xO1xuICAgIGxldCBpc1JlZHVjZWRFY2hlbG9uRm9ybSA9IHRydWU7XG4gICAgbGV0IGNoZWNrZWQgPSBmYWxzZTtcbiAgICB3aGlsZSAoaSA8IHRoaXMucm93cyAmJiBpc1JlZHVjZWRFY2hlbG9uRm9ybSkge1xuICAgICAgaiA9IDA7XG4gICAgICBjaGVja2VkID0gZmFsc2U7XG4gICAgICB3aGlsZSAoaiA8IHRoaXMuY29sdW1ucyAmJiBjaGVja2VkID09PSBmYWxzZSkge1xuICAgICAgICBpZiAodGhpcy5nZXQoaSwgaikgPT09IDApIHtcbiAgICAgICAgICBqKys7XG4gICAgICAgIH0gZWxzZSBpZiAodGhpcy5nZXQoaSwgaikgPT09IDEgJiYgaiA+IHByZXZpb3VzQ29sdW1uKSB7XG4gICAgICAgICAgY2hlY2tlZCA9IHRydWU7XG4gICAgICAgICAgcHJldmlvdXNDb2x1bW4gPSBqO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgIGlzUmVkdWNlZEVjaGVsb25Gb3JtID0gZmFsc2U7XG4gICAgICAgICAgY2hlY2tlZCA9IHRydWU7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIGZvciAobGV0IGsgPSBqICsgMTsgayA8IHRoaXMucm93czsgaysrKSB7XG4gICAgICAgIGlmICh0aGlzLmdldChpLCBrKSAhPT0gMCkge1xuICAgICAgICAgIGlzUmVkdWNlZEVjaGVsb25Gb3JtID0gZmFsc2U7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIGkrKztcbiAgICB9XG4gICAgcmV0dXJuIGlzUmVkdWNlZEVjaGVsb25Gb3JtO1xuICB9XG5cbiAgZWNoZWxvbkZvcm0oKSB7XG4gICAgbGV0IHJlc3VsdCA9IHRoaXMuY2xvbmUoKTtcbiAgICBsZXQgaCA9IDA7XG4gICAgbGV0IGsgPSAwO1xuICAgIHdoaWxlIChoIDwgcmVzdWx0LnJvd3MgJiYgayA8IHJlc3VsdC5jb2x1bW5zKSB7XG4gICAgICBsZXQgaU1heCA9IGg7XG4gICAgICBmb3IgKGxldCBpID0gaDsgaSA8IHJlc3VsdC5yb3dzOyBpKyspIHtcbiAgICAgICAgaWYgKHJlc3VsdC5nZXQoaSwgaykgPiByZXN1bHQuZ2V0KGlNYXgsIGspKSB7XG4gICAgICAgICAgaU1heCA9IGk7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIGlmIChyZXN1bHQuZ2V0KGlNYXgsIGspID09PSAwKSB7XG4gICAgICAgIGsrKztcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIHJlc3VsdC5zd2FwUm93cyhoLCBpTWF4KTtcbiAgICAgICAgbGV0IHRtcCA9IHJlc3VsdC5nZXQoaCwgayk7XG4gICAgICAgIGZvciAobGV0IGogPSBrOyBqIDwgcmVzdWx0LmNvbHVtbnM7IGorKykge1xuICAgICAgICAgIHJlc3VsdC5zZXQoaCwgaiwgcmVzdWx0LmdldChoLCBqKSAvIHRtcCk7XG4gICAgICAgIH1cbiAgICAgICAgZm9yIChsZXQgaSA9IGggKyAxOyBpIDwgcmVzdWx0LnJvd3M7IGkrKykge1xuICAgICAgICAgIGxldCBmYWN0b3IgPSByZXN1bHQuZ2V0KGksIGspIC8gcmVzdWx0LmdldChoLCBrKTtcbiAgICAgICAgICByZXN1bHQuc2V0KGksIGssIDApO1xuICAgICAgICAgIGZvciAobGV0IGogPSBrICsgMTsgaiA8IHJlc3VsdC5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgICAgIHJlc3VsdC5zZXQoaSwgaiwgcmVzdWx0LmdldChpLCBqKSAtIHJlc3VsdC5nZXQoaCwgaikgKiBmYWN0b3IpO1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICBoKys7XG4gICAgICAgIGsrKztcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHJlc3VsdDtcbiAgfVxuXG4gIHJlZHVjZWRFY2hlbG9uRm9ybSgpIHtcbiAgICBsZXQgcmVzdWx0ID0gdGhpcy5lY2hlbG9uRm9ybSgpO1xuICAgIGxldCBtID0gcmVzdWx0LmNvbHVtbnM7XG4gICAgbGV0IG4gPSByZXN1bHQucm93cztcbiAgICBsZXQgaCA9IG4gLSAxO1xuICAgIHdoaWxlIChoID49IDApIHtcbiAgICAgIGlmIChyZXN1bHQubWF4Um93KGgpID09PSAwKSB7XG4gICAgICAgIGgtLTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIGxldCBwID0gMDtcbiAgICAgICAgbGV0IHBpdm90ID0gZmFsc2U7XG4gICAgICAgIHdoaWxlIChwIDwgbiAmJiBwaXZvdCA9PT0gZmFsc2UpIHtcbiAgICAgICAgICBpZiAocmVzdWx0LmdldChoLCBwKSA9PT0gMSkge1xuICAgICAgICAgICAgcGl2b3QgPSB0cnVlO1xuICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICBwKys7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgaDsgaSsrKSB7XG4gICAgICAgICAgbGV0IGZhY3RvciA9IHJlc3VsdC5nZXQoaSwgcCk7XG4gICAgICAgICAgZm9yIChsZXQgaiA9IHA7IGogPCBtOyBqKyspIHtcbiAgICAgICAgICAgIGxldCB0bXAgPSByZXN1bHQuZ2V0KGksIGopIC0gZmFjdG9yICogcmVzdWx0LmdldChoLCBqKTtcbiAgICAgICAgICAgIHJlc3VsdC5zZXQoaSwgaiwgdG1wKTtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgaC0tO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gcmVzdWx0O1xuICB9XG5cbiAgc2V0KCkge1xuICAgIHRocm93IG5ldyBFcnJvcignc2V0IG1ldGhvZCBpcyB1bmltcGxlbWVudGVkJyk7XG4gIH1cblxuICBnZXQoKSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKCdnZXQgbWV0aG9kIGlzIHVuaW1wbGVtZW50ZWQnKTtcbiAgfVxuXG4gIHJlcGVhdChvcHRpb25zID0ge30pIHtcbiAgICBpZiAodHlwZW9mIG9wdGlvbnMgIT09ICdvYmplY3QnKSB7XG4gICAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdvcHRpb25zIG11c3QgYmUgYW4gb2JqZWN0Jyk7XG4gICAgfVxuICAgIGNvbnN0IHsgcm93cyA9IDEsIGNvbHVtbnMgPSAxIH0gPSBvcHRpb25zO1xuICAgIGlmICghTnVtYmVyLmlzSW50ZWdlcihyb3dzKSB8fCByb3dzIDw9IDApIHtcbiAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ3Jvd3MgbXVzdCBiZSBhIHBvc2l0aXZlIGludGVnZXInKTtcbiAgICB9XG4gICAgaWYgKCFOdW1iZXIuaXNJbnRlZ2VyKGNvbHVtbnMpIHx8IGNvbHVtbnMgPD0gMCkge1xuICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignY29sdW1ucyBtdXN0IGJlIGEgcG9zaXRpdmUgaW50ZWdlcicpO1xuICAgIH1cbiAgICBsZXQgbWF0cml4ID0gbmV3IE1hdHJpeCh0aGlzLnJvd3MgKiByb3dzLCB0aGlzLmNvbHVtbnMgKiBjb2x1bW5zKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBjb2x1bW5zOyBqKyspIHtcbiAgICAgICAgbWF0cml4LnNldFN1Yk1hdHJpeCh0aGlzLCB0aGlzLnJvd3MgKiBpLCB0aGlzLmNvbHVtbnMgKiBqKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIG1hdHJpeDtcbiAgfVxuXG4gIGZpbGwodmFsdWUpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHZhbHVlKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBuZWcoKSB7XG4gICAgcmV0dXJuIHRoaXMubXVsUygtMSk7XG4gIH1cblxuICBnZXRSb3coaW5kZXgpIHtcbiAgICBjaGVja1Jvd0luZGV4KHRoaXMsIGluZGV4KTtcbiAgICBsZXQgcm93ID0gW107XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLmNvbHVtbnM7IGkrKykge1xuICAgICAgcm93LnB1c2godGhpcy5nZXQoaW5kZXgsIGkpKTtcbiAgICB9XG4gICAgcmV0dXJuIHJvdztcbiAgfVxuXG4gIGdldFJvd1ZlY3RvcihpbmRleCkge1xuICAgIHJldHVybiBNYXRyaXgucm93VmVjdG9yKHRoaXMuZ2V0Um93KGluZGV4KSk7XG4gIH1cblxuICBzZXRSb3coaW5kZXgsIGFycmF5KSB7XG4gICAgY2hlY2tSb3dJbmRleCh0aGlzLCBpbmRleCk7XG4gICAgYXJyYXkgPSBjaGVja1Jvd1ZlY3Rvcih0aGlzLCBhcnJheSk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLmNvbHVtbnM7IGkrKykge1xuICAgICAgdGhpcy5zZXQoaW5kZXgsIGksIGFycmF5W2ldKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBzd2FwUm93cyhyb3cxLCByb3cyKSB7XG4gICAgY2hlY2tSb3dJbmRleCh0aGlzLCByb3cxKTtcbiAgICBjaGVja1Jvd0luZGV4KHRoaXMsIHJvdzIpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5jb2x1bW5zOyBpKyspIHtcbiAgICAgIGxldCB0ZW1wID0gdGhpcy5nZXQocm93MSwgaSk7XG4gICAgICB0aGlzLnNldChyb3cxLCBpLCB0aGlzLmdldChyb3cyLCBpKSk7XG4gICAgICB0aGlzLnNldChyb3cyLCBpLCB0ZW1wKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBnZXRDb2x1bW4oaW5kZXgpIHtcbiAgICBjaGVja0NvbHVtbkluZGV4KHRoaXMsIGluZGV4KTtcbiAgICBsZXQgY29sdW1uID0gW107XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgY29sdW1uLnB1c2godGhpcy5nZXQoaSwgaW5kZXgpKTtcbiAgICB9XG4gICAgcmV0dXJuIGNvbHVtbjtcbiAgfVxuXG4gIGdldENvbHVtblZlY3RvcihpbmRleCkge1xuICAgIHJldHVybiBNYXRyaXguY29sdW1uVmVjdG9yKHRoaXMuZ2V0Q29sdW1uKGluZGV4KSk7XG4gIH1cblxuICBzZXRDb2x1bW4oaW5kZXgsIGFycmF5KSB7XG4gICAgY2hlY2tDb2x1bW5JbmRleCh0aGlzLCBpbmRleCk7XG4gICAgYXJyYXkgPSBjaGVja0NvbHVtblZlY3Rvcih0aGlzLCBhcnJheSk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgdGhpcy5zZXQoaSwgaW5kZXgsIGFycmF5W2ldKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBzd2FwQ29sdW1ucyhjb2x1bW4xLCBjb2x1bW4yKSB7XG4gICAgY2hlY2tDb2x1bW5JbmRleCh0aGlzLCBjb2x1bW4xKTtcbiAgICBjaGVja0NvbHVtbkluZGV4KHRoaXMsIGNvbHVtbjIpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGxldCB0ZW1wID0gdGhpcy5nZXQoaSwgY29sdW1uMSk7XG4gICAgICB0aGlzLnNldChpLCBjb2x1bW4xLCB0aGlzLmdldChpLCBjb2x1bW4yKSk7XG4gICAgICB0aGlzLnNldChpLCBjb2x1bW4yLCB0ZW1wKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBhZGRSb3dWZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tSb3dWZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopICsgdmVjdG9yW2pdKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBzdWJSb3dWZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tSb3dWZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIC0gdmVjdG9yW2pdKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBtdWxSb3dWZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tSb3dWZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopICogdmVjdG9yW2pdKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBkaXZSb3dWZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tSb3dWZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIC8gdmVjdG9yW2pdKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBhZGRDb2x1bW5WZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tDb2x1bW5WZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopICsgdmVjdG9yW2ldKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBzdWJDb2x1bW5WZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tDb2x1bW5WZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIC0gdmVjdG9yW2ldKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBtdWxDb2x1bW5WZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tDb2x1bW5WZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopICogdmVjdG9yW2ldKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBkaXZDb2x1bW5WZWN0b3IodmVjdG9yKSB7XG4gICAgdmVjdG9yID0gY2hlY2tDb2x1bW5WZWN0b3IodGhpcywgdmVjdG9yKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHRoaXMuZ2V0KGksIGopIC8gdmVjdG9yW2ldKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBtdWxSb3coaW5kZXgsIHZhbHVlKSB7XG4gICAgY2hlY2tSb3dJbmRleCh0aGlzLCBpbmRleCk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLmNvbHVtbnM7IGkrKykge1xuICAgICAgdGhpcy5zZXQoaW5kZXgsIGksIHRoaXMuZ2V0KGluZGV4LCBpKSAqIHZhbHVlKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBtdWxDb2x1bW4oaW5kZXgsIHZhbHVlKSB7XG4gICAgY2hlY2tDb2x1bW5JbmRleCh0aGlzLCBpbmRleCk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgdGhpcy5zZXQoaSwgaW5kZXgsIHRoaXMuZ2V0KGksIGluZGV4KSAqIHZhbHVlKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBtYXgoYnkpIHtcbiAgICBpZiAodGhpcy5pc0VtcHR5KCkpIHtcbiAgICAgIHJldHVybiBOYU47XG4gICAgfVxuICAgIHN3aXRjaCAoYnkpIHtcbiAgICAgIGNhc2UgJ3Jvdyc6IHtcbiAgICAgICAgY29uc3QgbWF4ID0gbmV3IEFycmF5KHRoaXMucm93cykuZmlsbChOdW1iZXIuTkVHQVRJVkVfSU5GSU5JVFkpO1xuICAgICAgICBmb3IgKGxldCByb3cgPSAwOyByb3cgPCB0aGlzLnJvd3M7IHJvdysrKSB7XG4gICAgICAgICAgZm9yIChsZXQgY29sdW1uID0gMDsgY29sdW1uIDwgdGhpcy5jb2x1bW5zOyBjb2x1bW4rKykge1xuICAgICAgICAgICAgaWYgKHRoaXMuZ2V0KHJvdywgY29sdW1uKSA+IG1heFtyb3ddKSB7XG4gICAgICAgICAgICAgIG1heFtyb3ddID0gdGhpcy5nZXQocm93LCBjb2x1bW4pO1xuICAgICAgICAgICAgfVxuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICByZXR1cm4gbWF4O1xuICAgICAgfVxuICAgICAgY2FzZSAnY29sdW1uJzoge1xuICAgICAgICBjb25zdCBtYXggPSBuZXcgQXJyYXkodGhpcy5jb2x1bW5zKS5maWxsKE51bWJlci5ORUdBVElWRV9JTkZJTklUWSk7XG4gICAgICAgIGZvciAobGV0IHJvdyA9IDA7IHJvdyA8IHRoaXMucm93czsgcm93KyspIHtcbiAgICAgICAgICBmb3IgKGxldCBjb2x1bW4gPSAwOyBjb2x1bW4gPCB0aGlzLmNvbHVtbnM7IGNvbHVtbisrKSB7XG4gICAgICAgICAgICBpZiAodGhpcy5nZXQocm93LCBjb2x1bW4pID4gbWF4W2NvbHVtbl0pIHtcbiAgICAgICAgICAgICAgbWF4W2NvbHVtbl0gPSB0aGlzLmdldChyb3csIGNvbHVtbik7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIHJldHVybiBtYXg7XG4gICAgICB9XG4gICAgICBjYXNlIHVuZGVmaW5lZDoge1xuICAgICAgICBsZXQgbWF4ID0gdGhpcy5nZXQoMCwgMCk7XG4gICAgICAgIGZvciAobGV0IHJvdyA9IDA7IHJvdyA8IHRoaXMucm93czsgcm93KyspIHtcbiAgICAgICAgICBmb3IgKGxldCBjb2x1bW4gPSAwOyBjb2x1bW4gPCB0aGlzLmNvbHVtbnM7IGNvbHVtbisrKSB7XG4gICAgICAgICAgICBpZiAodGhpcy5nZXQocm93LCBjb2x1bW4pID4gbWF4KSB7XG4gICAgICAgICAgICAgIG1heCA9IHRoaXMuZ2V0KHJvdywgY29sdW1uKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIG1heDtcbiAgICAgIH1cbiAgICAgIGRlZmF1bHQ6XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihgaW52YWxpZCBvcHRpb246ICR7Ynl9YCk7XG4gICAgfVxuICB9XG5cbiAgbWF4SW5kZXgoKSB7XG4gICAgY2hlY2tOb25FbXB0eSh0aGlzKTtcbiAgICBsZXQgdiA9IHRoaXMuZ2V0KDAsIDApO1xuICAgIGxldCBpZHggPSBbMCwgMF07XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICBpZiAodGhpcy5nZXQoaSwgaikgPiB2KSB7XG4gICAgICAgICAgdiA9IHRoaXMuZ2V0KGksIGopO1xuICAgICAgICAgIGlkeFswXSA9IGk7XG4gICAgICAgICAgaWR4WzFdID0gajtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gaWR4O1xuICB9XG5cbiAgbWluKGJ5KSB7XG4gICAgaWYgKHRoaXMuaXNFbXB0eSgpKSB7XG4gICAgICByZXR1cm4gTmFOO1xuICAgIH1cblxuICAgIHN3aXRjaCAoYnkpIHtcbiAgICAgIGNhc2UgJ3Jvdyc6IHtcbiAgICAgICAgY29uc3QgbWluID0gbmV3IEFycmF5KHRoaXMucm93cykuZmlsbChOdW1iZXIuUE9TSVRJVkVfSU5GSU5JVFkpO1xuICAgICAgICBmb3IgKGxldCByb3cgPSAwOyByb3cgPCB0aGlzLnJvd3M7IHJvdysrKSB7XG4gICAgICAgICAgZm9yIChsZXQgY29sdW1uID0gMDsgY29sdW1uIDwgdGhpcy5jb2x1bW5zOyBjb2x1bW4rKykge1xuICAgICAgICAgICAgaWYgKHRoaXMuZ2V0KHJvdywgY29sdW1uKSA8IG1pbltyb3ddKSB7XG4gICAgICAgICAgICAgIG1pbltyb3ddID0gdGhpcy5nZXQocm93LCBjb2x1bW4pO1xuICAgICAgICAgICAgfVxuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICByZXR1cm4gbWluO1xuICAgICAgfVxuICAgICAgY2FzZSAnY29sdW1uJzoge1xuICAgICAgICBjb25zdCBtaW4gPSBuZXcgQXJyYXkodGhpcy5jb2x1bW5zKS5maWxsKE51bWJlci5QT1NJVElWRV9JTkZJTklUWSk7XG4gICAgICAgIGZvciAobGV0IHJvdyA9IDA7IHJvdyA8IHRoaXMucm93czsgcm93KyspIHtcbiAgICAgICAgICBmb3IgKGxldCBjb2x1bW4gPSAwOyBjb2x1bW4gPCB0aGlzLmNvbHVtbnM7IGNvbHVtbisrKSB7XG4gICAgICAgICAgICBpZiAodGhpcy5nZXQocm93LCBjb2x1bW4pIDwgbWluW2NvbHVtbl0pIHtcbiAgICAgICAgICAgICAgbWluW2NvbHVtbl0gPSB0aGlzLmdldChyb3csIGNvbHVtbik7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIHJldHVybiBtaW47XG4gICAgICB9XG4gICAgICBjYXNlIHVuZGVmaW5lZDoge1xuICAgICAgICBsZXQgbWluID0gdGhpcy5nZXQoMCwgMCk7XG4gICAgICAgIGZvciAobGV0IHJvdyA9IDA7IHJvdyA8IHRoaXMucm93czsgcm93KyspIHtcbiAgICAgICAgICBmb3IgKGxldCBjb2x1bW4gPSAwOyBjb2x1bW4gPCB0aGlzLmNvbHVtbnM7IGNvbHVtbisrKSB7XG4gICAgICAgICAgICBpZiAodGhpcy5nZXQocm93LCBjb2x1bW4pIDwgbWluKSB7XG4gICAgICAgICAgICAgIG1pbiA9IHRoaXMuZ2V0KHJvdywgY29sdW1uKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIG1pbjtcbiAgICAgIH1cbiAgICAgIGRlZmF1bHQ6XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihgaW52YWxpZCBvcHRpb246ICR7Ynl9YCk7XG4gICAgfVxuICB9XG5cbiAgbWluSW5kZXgoKSB7XG4gICAgY2hlY2tOb25FbXB0eSh0aGlzKTtcbiAgICBsZXQgdiA9IHRoaXMuZ2V0KDAsIDApO1xuICAgIGxldCBpZHggPSBbMCwgMF07XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICBpZiAodGhpcy5nZXQoaSwgaikgPCB2KSB7XG4gICAgICAgICAgdiA9IHRoaXMuZ2V0KGksIGopO1xuICAgICAgICAgIGlkeFswXSA9IGk7XG4gICAgICAgICAgaWR4WzFdID0gajtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gaWR4O1xuICB9XG5cbiAgbWF4Um93KHJvdykge1xuICAgIGNoZWNrUm93SW5kZXgodGhpcywgcm93KTtcbiAgICBpZiAodGhpcy5pc0VtcHR5KCkpIHtcbiAgICAgIHJldHVybiBOYU47XG4gICAgfVxuICAgIGxldCB2ID0gdGhpcy5nZXQocm93LCAwKTtcbiAgICBmb3IgKGxldCBpID0gMTsgaSA8IHRoaXMuY29sdW1uczsgaSsrKSB7XG4gICAgICBpZiAodGhpcy5nZXQocm93LCBpKSA+IHYpIHtcbiAgICAgICAgdiA9IHRoaXMuZ2V0KHJvdywgaSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB2O1xuICB9XG5cbiAgbWF4Um93SW5kZXgocm93KSB7XG4gICAgY2hlY2tSb3dJbmRleCh0aGlzLCByb3cpO1xuICAgIGNoZWNrTm9uRW1wdHkodGhpcyk7XG4gICAgbGV0IHYgPSB0aGlzLmdldChyb3csIDApO1xuICAgIGxldCBpZHggPSBbcm93LCAwXTtcbiAgICBmb3IgKGxldCBpID0gMTsgaSA8IHRoaXMuY29sdW1uczsgaSsrKSB7XG4gICAgICBpZiAodGhpcy5nZXQocm93LCBpKSA+IHYpIHtcbiAgICAgICAgdiA9IHRoaXMuZ2V0KHJvdywgaSk7XG4gICAgICAgIGlkeFsxXSA9IGk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBpZHg7XG4gIH1cblxuICBtaW5Sb3cocm93KSB7XG4gICAgY2hlY2tSb3dJbmRleCh0aGlzLCByb3cpO1xuICAgIGlmICh0aGlzLmlzRW1wdHkoKSkge1xuICAgICAgcmV0dXJuIE5hTjtcbiAgICB9XG4gICAgbGV0IHYgPSB0aGlzLmdldChyb3csIDApO1xuICAgIGZvciAobGV0IGkgPSAxOyBpIDwgdGhpcy5jb2x1bW5zOyBpKyspIHtcbiAgICAgIGlmICh0aGlzLmdldChyb3csIGkpIDwgdikge1xuICAgICAgICB2ID0gdGhpcy5nZXQocm93LCBpKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHY7XG4gIH1cblxuICBtaW5Sb3dJbmRleChyb3cpIHtcbiAgICBjaGVja1Jvd0luZGV4KHRoaXMsIHJvdyk7XG4gICAgY2hlY2tOb25FbXB0eSh0aGlzKTtcbiAgICBsZXQgdiA9IHRoaXMuZ2V0KHJvdywgMCk7XG4gICAgbGV0IGlkeCA9IFtyb3csIDBdO1xuICAgIGZvciAobGV0IGkgPSAxOyBpIDwgdGhpcy5jb2x1bW5zOyBpKyspIHtcbiAgICAgIGlmICh0aGlzLmdldChyb3csIGkpIDwgdikge1xuICAgICAgICB2ID0gdGhpcy5nZXQocm93LCBpKTtcbiAgICAgICAgaWR4WzFdID0gaTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIGlkeDtcbiAgfVxuXG4gIG1heENvbHVtbihjb2x1bW4pIHtcbiAgICBjaGVja0NvbHVtbkluZGV4KHRoaXMsIGNvbHVtbik7XG4gICAgaWYgKHRoaXMuaXNFbXB0eSgpKSB7XG4gICAgICByZXR1cm4gTmFOO1xuICAgIH1cbiAgICBsZXQgdiA9IHRoaXMuZ2V0KDAsIGNvbHVtbik7XG4gICAgZm9yIChsZXQgaSA9IDE7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgaWYgKHRoaXMuZ2V0KGksIGNvbHVtbikgPiB2KSB7XG4gICAgICAgIHYgPSB0aGlzLmdldChpLCBjb2x1bW4pO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdjtcbiAgfVxuXG4gIG1heENvbHVtbkluZGV4KGNvbHVtbikge1xuICAgIGNoZWNrQ29sdW1uSW5kZXgodGhpcywgY29sdW1uKTtcbiAgICBjaGVja05vbkVtcHR5KHRoaXMpO1xuICAgIGxldCB2ID0gdGhpcy5nZXQoMCwgY29sdW1uKTtcbiAgICBsZXQgaWR4ID0gWzAsIGNvbHVtbl07XG4gICAgZm9yIChsZXQgaSA9IDE7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgaWYgKHRoaXMuZ2V0KGksIGNvbHVtbikgPiB2KSB7XG4gICAgICAgIHYgPSB0aGlzLmdldChpLCBjb2x1bW4pO1xuICAgICAgICBpZHhbMF0gPSBpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gaWR4O1xuICB9XG5cbiAgbWluQ29sdW1uKGNvbHVtbikge1xuICAgIGNoZWNrQ29sdW1uSW5kZXgodGhpcywgY29sdW1uKTtcbiAgICBpZiAodGhpcy5pc0VtcHR5KCkpIHtcbiAgICAgIHJldHVybiBOYU47XG4gICAgfVxuICAgIGxldCB2ID0gdGhpcy5nZXQoMCwgY29sdW1uKTtcbiAgICBmb3IgKGxldCBpID0gMTsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBpZiAodGhpcy5nZXQoaSwgY29sdW1uKSA8IHYpIHtcbiAgICAgICAgdiA9IHRoaXMuZ2V0KGksIGNvbHVtbik7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB2O1xuICB9XG5cbiAgbWluQ29sdW1uSW5kZXgoY29sdW1uKSB7XG4gICAgY2hlY2tDb2x1bW5JbmRleCh0aGlzLCBjb2x1bW4pO1xuICAgIGNoZWNrTm9uRW1wdHkodGhpcyk7XG4gICAgbGV0IHYgPSB0aGlzLmdldCgwLCBjb2x1bW4pO1xuICAgIGxldCBpZHggPSBbMCwgY29sdW1uXTtcbiAgICBmb3IgKGxldCBpID0gMTsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBpZiAodGhpcy5nZXQoaSwgY29sdW1uKSA8IHYpIHtcbiAgICAgICAgdiA9IHRoaXMuZ2V0KGksIGNvbHVtbik7XG4gICAgICAgIGlkeFswXSA9IGk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBpZHg7XG4gIH1cblxuICBkaWFnKCkge1xuICAgIGxldCBtaW4gPSBNYXRoLm1pbih0aGlzLnJvd3MsIHRoaXMuY29sdW1ucyk7XG4gICAgbGV0IGRpYWcgPSBbXTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IG1pbjsgaSsrKSB7XG4gICAgICBkaWFnLnB1c2godGhpcy5nZXQoaSwgaSkpO1xuICAgIH1cbiAgICByZXR1cm4gZGlhZztcbiAgfVxuXG4gIG5vcm0odHlwZSA9ICdmcm9iZW5pdXMnKSB7XG4gICAgbGV0IHJlc3VsdCA9IDA7XG4gICAgaWYgKHR5cGUgPT09ICdtYXgnKSB7XG4gICAgICByZXR1cm4gdGhpcy5tYXgoKTtcbiAgICB9IGVsc2UgaWYgKHR5cGUgPT09ICdmcm9iZW5pdXMnKSB7XG4gICAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgdGhpcy5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgICByZXN1bHQgPSByZXN1bHQgKyB0aGlzLmdldChpLCBqKSAqIHRoaXMuZ2V0KGksIGopO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgICByZXR1cm4gTWF0aC5zcXJ0KHJlc3VsdCk7XG4gICAgfSBlbHNlIHtcbiAgICAgIHRocm93IG5ldyBSYW5nZUVycm9yKGB1bmtub3duIG5vcm0gdHlwZTogJHt0eXBlfWApO1xuICAgIH1cbiAgfVxuXG4gIGN1bXVsYXRpdmVTdW0oKSB7XG4gICAgbGV0IHN1bSA9IDA7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLnJvd3M7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICBzdW0gKz0gdGhpcy5nZXQoaSwgaik7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIHN1bSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xuICB9XG5cbiAgZG90KHZlY3RvcjIpIHtcbiAgICBpZiAoQWJzdHJhY3RNYXRyaXguaXNNYXRyaXgodmVjdG9yMikpIHZlY3RvcjIgPSB2ZWN0b3IyLnRvMURBcnJheSgpO1xuICAgIGxldCB2ZWN0b3IxID0gdGhpcy50bzFEQXJyYXkoKTtcbiAgICBpZiAodmVjdG9yMS5sZW5ndGggIT09IHZlY3RvcjIubGVuZ3RoKSB7XG4gICAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcigndmVjdG9ycyBkbyBub3QgaGF2ZSB0aGUgc2FtZSBzaXplJyk7XG4gICAgfVxuICAgIGxldCBkb3QgPSAwO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdmVjdG9yMS5sZW5ndGg7IGkrKykge1xuICAgICAgZG90ICs9IHZlY3RvcjFbaV0gKiB2ZWN0b3IyW2ldO1xuICAgIH1cbiAgICByZXR1cm4gZG90O1xuICB9XG5cbiAgbW11bChvdGhlcikge1xuICAgIG90aGVyID0gTWF0cml4LmNoZWNrTWF0cml4KG90aGVyKTtcblxuICAgIGxldCBtID0gdGhpcy5yb3dzO1xuICAgIGxldCBuID0gdGhpcy5jb2x1bW5zO1xuICAgIGxldCBwID0gb3RoZXIuY29sdW1ucztcblxuICAgIGxldCByZXN1bHQgPSBuZXcgTWF0cml4KG0sIHApO1xuXG4gICAgbGV0IEJjb2xqID0gbmV3IEZsb2F0NjRBcnJheShuKTtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IHA7IGorKykge1xuICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCBuOyBrKyspIHtcbiAgICAgICAgQmNvbGpba10gPSBvdGhlci5nZXQoaywgaik7XG4gICAgICB9XG5cbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbTsgaSsrKSB7XG4gICAgICAgIGxldCBzID0gMDtcbiAgICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCBuOyBrKyspIHtcbiAgICAgICAgICBzICs9IHRoaXMuZ2V0KGksIGspICogQmNvbGpba107XG4gICAgICAgIH1cblxuICAgICAgICByZXN1bHQuc2V0KGksIGosIHMpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gcmVzdWx0O1xuICB9XG5cbiAgc3RyYXNzZW4yeDIob3RoZXIpIHtcbiAgICBvdGhlciA9IE1hdHJpeC5jaGVja01hdHJpeChvdGhlcik7XG4gICAgbGV0IHJlc3VsdCA9IG5ldyBNYXRyaXgoMiwgMik7XG4gICAgY29uc3QgYTExID0gdGhpcy5nZXQoMCwgMCk7XG4gICAgY29uc3QgYjExID0gb3RoZXIuZ2V0KDAsIDApO1xuICAgIGNvbnN0IGExMiA9IHRoaXMuZ2V0KDAsIDEpO1xuICAgIGNvbnN0IGIxMiA9IG90aGVyLmdldCgwLCAxKTtcbiAgICBjb25zdCBhMjEgPSB0aGlzLmdldCgxLCAwKTtcbiAgICBjb25zdCBiMjEgPSBvdGhlci5nZXQoMSwgMCk7XG4gICAgY29uc3QgYTIyID0gdGhpcy5nZXQoMSwgMSk7XG4gICAgY29uc3QgYjIyID0gb3RoZXIuZ2V0KDEsIDEpO1xuXG4gICAgLy8gQ29tcHV0ZSBpbnRlcm1lZGlhdGUgdmFsdWVzLlxuICAgIGNvbnN0IG0xID0gKGExMSArIGEyMikgKiAoYjExICsgYjIyKTtcbiAgICBjb25zdCBtMiA9IChhMjEgKyBhMjIpICogYjExO1xuICAgIGNvbnN0IG0zID0gYTExICogKGIxMiAtIGIyMik7XG4gICAgY29uc3QgbTQgPSBhMjIgKiAoYjIxIC0gYjExKTtcbiAgICBjb25zdCBtNSA9IChhMTEgKyBhMTIpICogYjIyO1xuICAgIGNvbnN0IG02ID0gKGEyMSAtIGExMSkgKiAoYjExICsgYjEyKTtcbiAgICBjb25zdCBtNyA9IChhMTIgLSBhMjIpICogKGIyMSArIGIyMik7XG5cbiAgICAvLyBDb21iaW5lIGludGVybWVkaWF0ZSB2YWx1ZXMgaW50byB0aGUgb3V0cHV0LlxuICAgIGNvbnN0IGMwMCA9IG0xICsgbTQgLSBtNSArIG03O1xuICAgIGNvbnN0IGMwMSA9IG0zICsgbTU7XG4gICAgY29uc3QgYzEwID0gbTIgKyBtNDtcbiAgICBjb25zdCBjMTEgPSBtMSAtIG0yICsgbTMgKyBtNjtcblxuICAgIHJlc3VsdC5zZXQoMCwgMCwgYzAwKTtcbiAgICByZXN1bHQuc2V0KDAsIDEsIGMwMSk7XG4gICAgcmVzdWx0LnNldCgxLCAwLCBjMTApO1xuICAgIHJlc3VsdC5zZXQoMSwgMSwgYzExKTtcbiAgICByZXR1cm4gcmVzdWx0O1xuICB9XG5cbiAgc3RyYXNzZW4zeDMob3RoZXIpIHtcbiAgICBvdGhlciA9IE1hdHJpeC5jaGVja01hdHJpeChvdGhlcik7XG4gICAgbGV0IHJlc3VsdCA9IG5ldyBNYXRyaXgoMywgMyk7XG5cbiAgICBjb25zdCBhMDAgPSB0aGlzLmdldCgwLCAwKTtcbiAgICBjb25zdCBhMDEgPSB0aGlzLmdldCgwLCAxKTtcbiAgICBjb25zdCBhMDIgPSB0aGlzLmdldCgwLCAyKTtcbiAgICBjb25zdCBhMTAgPSB0aGlzLmdldCgxLCAwKTtcbiAgICBjb25zdCBhMTEgPSB0aGlzLmdldCgxLCAxKTtcbiAgICBjb25zdCBhMTIgPSB0aGlzLmdldCgxLCAyKTtcbiAgICBjb25zdCBhMjAgPSB0aGlzLmdldCgyLCAwKTtcbiAgICBjb25zdCBhMjEgPSB0aGlzLmdldCgyLCAxKTtcbiAgICBjb25zdCBhMjIgPSB0aGlzLmdldCgyLCAyKTtcblxuICAgIGNvbnN0IGIwMCA9IG90aGVyLmdldCgwLCAwKTtcbiAgICBjb25zdCBiMDEgPSBvdGhlci5nZXQoMCwgMSk7XG4gICAgY29uc3QgYjAyID0gb3RoZXIuZ2V0KDAsIDIpO1xuICAgIGNvbnN0IGIxMCA9IG90aGVyLmdldCgxLCAwKTtcbiAgICBjb25zdCBiMTEgPSBvdGhlci5nZXQoMSwgMSk7XG4gICAgY29uc3QgYjEyID0gb3RoZXIuZ2V0KDEsIDIpO1xuICAgIGNvbnN0IGIyMCA9IG90aGVyLmdldCgyLCAwKTtcbiAgICBjb25zdCBiMjEgPSBvdGhlci5nZXQoMiwgMSk7XG4gICAgY29uc3QgYjIyID0gb3RoZXIuZ2V0KDIsIDIpO1xuXG4gICAgY29uc3QgbTEgPSAoYTAwICsgYTAxICsgYTAyIC0gYTEwIC0gYTExIC0gYTIxIC0gYTIyKSAqIGIxMTtcbiAgICBjb25zdCBtMiA9IChhMDAgLSBhMTApICogKC1iMDEgKyBiMTEpO1xuICAgIGNvbnN0IG0zID0gYTExICogKC1iMDAgKyBiMDEgKyBiMTAgLSBiMTEgLSBiMTIgLSBiMjAgKyBiMjIpO1xuICAgIGNvbnN0IG00ID0gKC1hMDAgKyBhMTAgKyBhMTEpICogKGIwMCAtIGIwMSArIGIxMSk7XG4gICAgY29uc3QgbTUgPSAoYTEwICsgYTExKSAqICgtYjAwICsgYjAxKTtcbiAgICBjb25zdCBtNiA9IGEwMCAqIGIwMDtcbiAgICBjb25zdCBtNyA9ICgtYTAwICsgYTIwICsgYTIxKSAqIChiMDAgLSBiMDIgKyBiMTIpO1xuICAgIGNvbnN0IG04ID0gKC1hMDAgKyBhMjApICogKGIwMiAtIGIxMik7XG4gICAgY29uc3QgbTkgPSAoYTIwICsgYTIxKSAqICgtYjAwICsgYjAyKTtcbiAgICBjb25zdCBtMTAgPSAoYTAwICsgYTAxICsgYTAyIC0gYTExIC0gYTEyIC0gYTIwIC0gYTIxKSAqIGIxMjtcbiAgICBjb25zdCBtMTEgPSBhMjEgKiAoLWIwMCArIGIwMiArIGIxMCAtIGIxMSAtIGIxMiAtIGIyMCArIGIyMSk7XG4gICAgY29uc3QgbTEyID0gKC1hMDIgKyBhMjEgKyBhMjIpICogKGIxMSArIGIyMCAtIGIyMSk7XG4gICAgY29uc3QgbTEzID0gKGEwMiAtIGEyMikgKiAoYjExIC0gYjIxKTtcbiAgICBjb25zdCBtMTQgPSBhMDIgKiBiMjA7XG4gICAgY29uc3QgbTE1ID0gKGEyMSArIGEyMikgKiAoLWIyMCArIGIyMSk7XG4gICAgY29uc3QgbTE2ID0gKC1hMDIgKyBhMTEgKyBhMTIpICogKGIxMiArIGIyMCAtIGIyMik7XG4gICAgY29uc3QgbTE3ID0gKGEwMiAtIGExMikgKiAoYjEyIC0gYjIyKTtcbiAgICBjb25zdCBtMTggPSAoYTExICsgYTEyKSAqICgtYjIwICsgYjIyKTtcbiAgICBjb25zdCBtMTkgPSBhMDEgKiBiMTA7XG4gICAgY29uc3QgbTIwID0gYTEyICogYjIxO1xuICAgIGNvbnN0IG0yMSA9IGExMCAqIGIwMjtcbiAgICBjb25zdCBtMjIgPSBhMjAgKiBiMDE7XG4gICAgY29uc3QgbTIzID0gYTIyICogYjIyO1xuXG4gICAgY29uc3QgYzAwID0gbTYgKyBtMTQgKyBtMTk7XG4gICAgY29uc3QgYzAxID0gbTEgKyBtNCArIG01ICsgbTYgKyBtMTIgKyBtMTQgKyBtMTU7XG4gICAgY29uc3QgYzAyID0gbTYgKyBtNyArIG05ICsgbTEwICsgbTE0ICsgbTE2ICsgbTE4O1xuICAgIGNvbnN0IGMxMCA9IG0yICsgbTMgKyBtNCArIG02ICsgbTE0ICsgbTE2ICsgbTE3O1xuICAgIGNvbnN0IGMxMSA9IG0yICsgbTQgKyBtNSArIG02ICsgbTIwO1xuICAgIGNvbnN0IGMxMiA9IG0xNCArIG0xNiArIG0xNyArIG0xOCArIG0yMTtcbiAgICBjb25zdCBjMjAgPSBtNiArIG03ICsgbTggKyBtMTEgKyBtMTIgKyBtMTMgKyBtMTQ7XG4gICAgY29uc3QgYzIxID0gbTEyICsgbTEzICsgbTE0ICsgbTE1ICsgbTIyO1xuICAgIGNvbnN0IGMyMiA9IG02ICsgbTcgKyBtOCArIG05ICsgbTIzO1xuXG4gICAgcmVzdWx0LnNldCgwLCAwLCBjMDApO1xuICAgIHJlc3VsdC5zZXQoMCwgMSwgYzAxKTtcbiAgICByZXN1bHQuc2V0KDAsIDIsIGMwMik7XG4gICAgcmVzdWx0LnNldCgxLCAwLCBjMTApO1xuICAgIHJlc3VsdC5zZXQoMSwgMSwgYzExKTtcbiAgICByZXN1bHQuc2V0KDEsIDIsIGMxMik7XG4gICAgcmVzdWx0LnNldCgyLCAwLCBjMjApO1xuICAgIHJlc3VsdC5zZXQoMiwgMSwgYzIxKTtcbiAgICByZXN1bHQuc2V0KDIsIDIsIGMyMik7XG4gICAgcmV0dXJuIHJlc3VsdDtcbiAgfVxuXG4gIG1tdWxTdHJhc3Nlbih5KSB7XG4gICAgeSA9IE1hdHJpeC5jaGVja01hdHJpeCh5KTtcbiAgICBsZXQgeCA9IHRoaXMuY2xvbmUoKTtcbiAgICBsZXQgcjEgPSB4LnJvd3M7XG4gICAgbGV0IGMxID0geC5jb2x1bW5zO1xuICAgIGxldCByMiA9IHkucm93cztcbiAgICBsZXQgYzIgPSB5LmNvbHVtbnM7XG4gICAgaWYgKGMxICE9PSByMikge1xuICAgICAgLy8gZXNsaW50LWRpc2FibGUtbmV4dC1saW5lIG5vLWNvbnNvbGVcbiAgICAgIGNvbnNvbGUud2FybihcbiAgICAgICAgYE11bHRpcGx5aW5nICR7cjF9IHggJHtjMX0gYW5kICR7cjJ9IHggJHtjMn0gbWF0cml4OiBkaW1lbnNpb25zIGRvIG5vdCBtYXRjaC5gLFxuICAgICAgKTtcbiAgICB9XG5cbiAgICAvLyBQdXQgYSBtYXRyaXggaW50byB0aGUgdG9wIGxlZnQgb2YgYSBtYXRyaXggb2YgemVyb3MuXG4gICAgLy8gYHJvd3NgIGFuZCBgY29sc2AgYXJlIHRoZSBkaW1lbnNpb25zIG9mIHRoZSBvdXRwdXQgbWF0cml4LlxuICAgIGZ1bmN0aW9uIGVtYmVkKG1hdCwgcm93cywgY29scykge1xuICAgICAgbGV0IHIgPSBtYXQucm93cztcbiAgICAgIGxldCBjID0gbWF0LmNvbHVtbnM7XG4gICAgICBpZiAociA9PT0gcm93cyAmJiBjID09PSBjb2xzKSB7XG4gICAgICAgIHJldHVybiBtYXQ7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBsZXQgcmVzdWx0YXQgPSBBYnN0cmFjdE1hdHJpeC56ZXJvcyhyb3dzLCBjb2xzKTtcbiAgICAgICAgcmVzdWx0YXQgPSByZXN1bHRhdC5zZXRTdWJNYXRyaXgobWF0LCAwLCAwKTtcbiAgICAgICAgcmV0dXJuIHJlc3VsdGF0O1xuICAgICAgfVxuICAgIH1cblxuICAgIC8vIE1ha2Ugc3VyZSBib3RoIG1hdHJpY2VzIGFyZSB0aGUgc2FtZSBzaXplLlxuICAgIC8vIFRoaXMgaXMgZXhjbHVzaXZlbHkgZm9yIHNpbXBsaWNpdHk6XG4gICAgLy8gdGhpcyBhbGdvcml0aG0gY2FuIGJlIGltcGxlbWVudGVkIHdpdGggbWF0cmljZXMgb2YgZGlmZmVyZW50IHNpemVzLlxuXG4gICAgbGV0IHIgPSBNYXRoLm1heChyMSwgcjIpO1xuICAgIGxldCBjID0gTWF0aC5tYXgoYzEsIGMyKTtcbiAgICB4ID0gZW1iZWQoeCwgciwgYyk7XG4gICAgeSA9IGVtYmVkKHksIHIsIGMpO1xuXG4gICAgLy8gT3VyIHJlY3Vyc2l2ZSBtdWx0aXBsaWNhdGlvbiBmdW5jdGlvbi5cbiAgICBmdW5jdGlvbiBibG9ja011bHQoYSwgYiwgcm93cywgY29scykge1xuICAgICAgLy8gRm9yIHNtYWxsIG1hdHJpY2VzLCByZXNvcnQgdG8gbmFpdmUgbXVsdGlwbGljYXRpb24uXG4gICAgICBpZiAocm93cyA8PSA1MTIgfHwgY29scyA8PSA1MTIpIHtcbiAgICAgICAgcmV0dXJuIGEubW11bChiKTsgLy8gYSBpcyBlcXVpdmFsZW50IHRvIHRoaXNcbiAgICAgIH1cblxuICAgICAgLy8gQXBwbHkgZHluYW1pYyBwYWRkaW5nLlxuICAgICAgaWYgKHJvd3MgJSAyID09PSAxICYmIGNvbHMgJSAyID09PSAxKSB7XG4gICAgICAgIGEgPSBlbWJlZChhLCByb3dzICsgMSwgY29scyArIDEpO1xuICAgICAgICBiID0gZW1iZWQoYiwgcm93cyArIDEsIGNvbHMgKyAxKTtcbiAgICAgIH0gZWxzZSBpZiAocm93cyAlIDIgPT09IDEpIHtcbiAgICAgICAgYSA9IGVtYmVkKGEsIHJvd3MgKyAxLCBjb2xzKTtcbiAgICAgICAgYiA9IGVtYmVkKGIsIHJvd3MgKyAxLCBjb2xzKTtcbiAgICAgIH0gZWxzZSBpZiAoY29scyAlIDIgPT09IDEpIHtcbiAgICAgICAgYSA9IGVtYmVkKGEsIHJvd3MsIGNvbHMgKyAxKTtcbiAgICAgICAgYiA9IGVtYmVkKGIsIHJvd3MsIGNvbHMgKyAxKTtcbiAgICAgIH1cblxuICAgICAgbGV0IGhhbGZSb3dzID0gcGFyc2VJbnQoYS5yb3dzIC8gMiwgMTApO1xuICAgICAgbGV0IGhhbGZDb2xzID0gcGFyc2VJbnQoYS5jb2x1bW5zIC8gMiwgMTApO1xuICAgICAgLy8gU3ViZGl2aWRlIGlucHV0IG1hdHJpY2VzLlxuICAgICAgbGV0IGExMSA9IGEuc3ViTWF0cml4KDAsIGhhbGZSb3dzIC0gMSwgMCwgaGFsZkNvbHMgLSAxKTtcbiAgICAgIGxldCBiMTEgPSBiLnN1Yk1hdHJpeCgwLCBoYWxmUm93cyAtIDEsIDAsIGhhbGZDb2xzIC0gMSk7XG5cbiAgICAgIGxldCBhMTIgPSBhLnN1Yk1hdHJpeCgwLCBoYWxmUm93cyAtIDEsIGhhbGZDb2xzLCBhLmNvbHVtbnMgLSAxKTtcbiAgICAgIGxldCBiMTIgPSBiLnN1Yk1hdHJpeCgwLCBoYWxmUm93cyAtIDEsIGhhbGZDb2xzLCBiLmNvbHVtbnMgLSAxKTtcblxuICAgICAgbGV0IGEyMSA9IGEuc3ViTWF0cml4KGhhbGZSb3dzLCBhLnJvd3MgLSAxLCAwLCBoYWxmQ29scyAtIDEpO1xuICAgICAgbGV0IGIyMSA9IGIuc3ViTWF0cml4KGhhbGZSb3dzLCBiLnJvd3MgLSAxLCAwLCBoYWxmQ29scyAtIDEpO1xuXG4gICAgICBsZXQgYTIyID0gYS5zdWJNYXRyaXgoaGFsZlJvd3MsIGEucm93cyAtIDEsIGhhbGZDb2xzLCBhLmNvbHVtbnMgLSAxKTtcbiAgICAgIGxldCBiMjIgPSBiLnN1Yk1hdHJpeChoYWxmUm93cywgYi5yb3dzIC0gMSwgaGFsZkNvbHMsIGIuY29sdW1ucyAtIDEpO1xuXG4gICAgICAvLyBDb21wdXRlIGludGVybWVkaWF0ZSB2YWx1ZXMuXG4gICAgICBsZXQgbTEgPSBibG9ja011bHQoXG4gICAgICAgIEFic3RyYWN0TWF0cml4LmFkZChhMTEsIGEyMiksXG4gICAgICAgIEFic3RyYWN0TWF0cml4LmFkZChiMTEsIGIyMiksXG4gICAgICAgIGhhbGZSb3dzLFxuICAgICAgICBoYWxmQ29scyxcbiAgICAgICk7XG4gICAgICBsZXQgbTIgPSBibG9ja011bHQoQWJzdHJhY3RNYXRyaXguYWRkKGEyMSwgYTIyKSwgYjExLCBoYWxmUm93cywgaGFsZkNvbHMpO1xuICAgICAgbGV0IG0zID0gYmxvY2tNdWx0KGExMSwgQWJzdHJhY3RNYXRyaXguc3ViKGIxMiwgYjIyKSwgaGFsZlJvd3MsIGhhbGZDb2xzKTtcbiAgICAgIGxldCBtNCA9IGJsb2NrTXVsdChhMjIsIEFic3RyYWN0TWF0cml4LnN1YihiMjEsIGIxMSksIGhhbGZSb3dzLCBoYWxmQ29scyk7XG4gICAgICBsZXQgbTUgPSBibG9ja011bHQoQWJzdHJhY3RNYXRyaXguYWRkKGExMSwgYTEyKSwgYjIyLCBoYWxmUm93cywgaGFsZkNvbHMpO1xuICAgICAgbGV0IG02ID0gYmxvY2tNdWx0KFxuICAgICAgICBBYnN0cmFjdE1hdHJpeC5zdWIoYTIxLCBhMTEpLFxuICAgICAgICBBYnN0cmFjdE1hdHJpeC5hZGQoYjExLCBiMTIpLFxuICAgICAgICBoYWxmUm93cyxcbiAgICAgICAgaGFsZkNvbHMsXG4gICAgICApO1xuICAgICAgbGV0IG03ID0gYmxvY2tNdWx0KFxuICAgICAgICBBYnN0cmFjdE1hdHJpeC5zdWIoYTEyLCBhMjIpLFxuICAgICAgICBBYnN0cmFjdE1hdHJpeC5hZGQoYjIxLCBiMjIpLFxuICAgICAgICBoYWxmUm93cyxcbiAgICAgICAgaGFsZkNvbHMsXG4gICAgICApO1xuXG4gICAgICAvLyBDb21iaW5lIGludGVybWVkaWF0ZSB2YWx1ZXMgaW50byB0aGUgb3V0cHV0LlxuICAgICAgbGV0IGMxMSA9IEFic3RyYWN0TWF0cml4LmFkZChtMSwgbTQpO1xuICAgICAgYzExLnN1YihtNSk7XG4gICAgICBjMTEuYWRkKG03KTtcbiAgICAgIGxldCBjMTIgPSBBYnN0cmFjdE1hdHJpeC5hZGQobTMsIG01KTtcbiAgICAgIGxldCBjMjEgPSBBYnN0cmFjdE1hdHJpeC5hZGQobTIsIG00KTtcbiAgICAgIGxldCBjMjIgPSBBYnN0cmFjdE1hdHJpeC5zdWIobTEsIG0yKTtcbiAgICAgIGMyMi5hZGQobTMpO1xuICAgICAgYzIyLmFkZChtNik7XG5cbiAgICAgIC8vIENyb3Agb3V0cHV0IHRvIHRoZSBkZXNpcmVkIHNpemUgKHVuZG8gZHluYW1pYyBwYWRkaW5nKS5cbiAgICAgIGxldCByZXN1bHRhdCA9IEFic3RyYWN0TWF0cml4Lnplcm9zKDIgKiBjMTEucm93cywgMiAqIGMxMS5jb2x1bW5zKTtcbiAgICAgIHJlc3VsdGF0ID0gcmVzdWx0YXQuc2V0U3ViTWF0cml4KGMxMSwgMCwgMCk7XG4gICAgICByZXN1bHRhdCA9IHJlc3VsdGF0LnNldFN1Yk1hdHJpeChjMTIsIGMxMS5yb3dzLCAwKTtcbiAgICAgIHJlc3VsdGF0ID0gcmVzdWx0YXQuc2V0U3ViTWF0cml4KGMyMSwgMCwgYzExLmNvbHVtbnMpO1xuICAgICAgcmVzdWx0YXQgPSByZXN1bHRhdC5zZXRTdWJNYXRyaXgoYzIyLCBjMTEucm93cywgYzExLmNvbHVtbnMpO1xuICAgICAgcmV0dXJuIHJlc3VsdGF0LnN1Yk1hdHJpeCgwLCByb3dzIC0gMSwgMCwgY29scyAtIDEpO1xuICAgIH1cblxuICAgIHJldHVybiBibG9ja011bHQoeCwgeSwgciwgYyk7XG4gIH1cblxuICBzY2FsZVJvd3Mob3B0aW9ucyA9IHt9KSB7XG4gICAgaWYgKHR5cGVvZiBvcHRpb25zICE9PSAnb2JqZWN0Jykge1xuICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignb3B0aW9ucyBtdXN0IGJlIGFuIG9iamVjdCcpO1xuICAgIH1cbiAgICBjb25zdCB7IG1pbiA9IDAsIG1heCA9IDEgfSA9IG9wdGlvbnM7XG4gICAgaWYgKCFOdW1iZXIuaXNGaW5pdGUobWluKSkgdGhyb3cgbmV3IFR5cGVFcnJvcignbWluIG11c3QgYmUgYSBudW1iZXInKTtcbiAgICBpZiAoIU51bWJlci5pc0Zpbml0ZShtYXgpKSB0aHJvdyBuZXcgVHlwZUVycm9yKCdtYXggbXVzdCBiZSBhIG51bWJlcicpO1xuICAgIGlmIChtaW4gPj0gbWF4KSB0aHJvdyBuZXcgUmFuZ2VFcnJvcignbWluIG11c3QgYmUgc21hbGxlciB0aGFuIG1heCcpO1xuICAgIGxldCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KHRoaXMucm93cywgdGhpcy5jb2x1bW5zKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBjb25zdCByb3cgPSB0aGlzLmdldFJvdyhpKTtcbiAgICAgIGlmIChyb3cubGVuZ3RoID4gMCkge1xuICAgICAgICByZXNjYWxlKHJvdywgeyBtaW4sIG1heCwgb3V0cHV0OiByb3cgfSk7XG4gICAgICB9XG4gICAgICBuZXdNYXRyaXguc2V0Um93KGksIHJvdyk7XG4gICAgfVxuICAgIHJldHVybiBuZXdNYXRyaXg7XG4gIH1cblxuICBzY2FsZUNvbHVtbnMob3B0aW9ucyA9IHt9KSB7XG4gICAgaWYgKHR5cGVvZiBvcHRpb25zICE9PSAnb2JqZWN0Jykge1xuICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignb3B0aW9ucyBtdXN0IGJlIGFuIG9iamVjdCcpO1xuICAgIH1cbiAgICBjb25zdCB7IG1pbiA9IDAsIG1heCA9IDEgfSA9IG9wdGlvbnM7XG4gICAgaWYgKCFOdW1iZXIuaXNGaW5pdGUobWluKSkgdGhyb3cgbmV3IFR5cGVFcnJvcignbWluIG11c3QgYmUgYSBudW1iZXInKTtcbiAgICBpZiAoIU51bWJlci5pc0Zpbml0ZShtYXgpKSB0aHJvdyBuZXcgVHlwZUVycm9yKCdtYXggbXVzdCBiZSBhIG51bWJlcicpO1xuICAgIGlmIChtaW4gPj0gbWF4KSB0aHJvdyBuZXcgUmFuZ2VFcnJvcignbWluIG11c3QgYmUgc21hbGxlciB0aGFuIG1heCcpO1xuICAgIGxldCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KHRoaXMucm93cywgdGhpcy5jb2x1bW5zKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMuY29sdW1uczsgaSsrKSB7XG4gICAgICBjb25zdCBjb2x1bW4gPSB0aGlzLmdldENvbHVtbihpKTtcbiAgICAgIGlmIChjb2x1bW4ubGVuZ3RoKSB7XG4gICAgICAgIHJlc2NhbGUoY29sdW1uLCB7XG4gICAgICAgICAgbWluOiBtaW4sXG4gICAgICAgICAgbWF4OiBtYXgsXG4gICAgICAgICAgb3V0cHV0OiBjb2x1bW4sXG4gICAgICAgIH0pO1xuICAgICAgfVxuICAgICAgbmV3TWF0cml4LnNldENvbHVtbihpLCBjb2x1bW4pO1xuICAgIH1cbiAgICByZXR1cm4gbmV3TWF0cml4O1xuICB9XG5cbiAgZmxpcFJvd3MoKSB7XG4gICAgY29uc3QgbWlkZGxlID0gTWF0aC5jZWlsKHRoaXMuY29sdW1ucyAvIDIpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgbWlkZGxlOyBqKyspIHtcbiAgICAgICAgbGV0IGZpcnN0ID0gdGhpcy5nZXQoaSwgaik7XG4gICAgICAgIGxldCBsYXN0ID0gdGhpcy5nZXQoaSwgdGhpcy5jb2x1bW5zIC0gMSAtIGopO1xuICAgICAgICB0aGlzLnNldChpLCBqLCBsYXN0KTtcbiAgICAgICAgdGhpcy5zZXQoaSwgdGhpcy5jb2x1bW5zIC0gMSAtIGosIGZpcnN0KTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBmbGlwQ29sdW1ucygpIHtcbiAgICBjb25zdCBtaWRkbGUgPSBNYXRoLmNlaWwodGhpcy5yb3dzIC8gMik7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCBtaWRkbGU7IGkrKykge1xuICAgICAgICBsZXQgZmlyc3QgPSB0aGlzLmdldChpLCBqKTtcbiAgICAgICAgbGV0IGxhc3QgPSB0aGlzLmdldCh0aGlzLnJvd3MgLSAxIC0gaSwgaik7XG4gICAgICAgIHRoaXMuc2V0KGksIGosIGxhc3QpO1xuICAgICAgICB0aGlzLnNldCh0aGlzLnJvd3MgLSAxIC0gaSwgaiwgZmlyc3QpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIGtyb25lY2tlclByb2R1Y3Qob3RoZXIpIHtcbiAgICBvdGhlciA9IE1hdHJpeC5jaGVja01hdHJpeChvdGhlcik7XG5cbiAgICBsZXQgbSA9IHRoaXMucm93cztcbiAgICBsZXQgbiA9IHRoaXMuY29sdW1ucztcbiAgICBsZXQgcCA9IG90aGVyLnJvd3M7XG4gICAgbGV0IHEgPSBvdGhlci5jb2x1bW5zO1xuXG4gICAgbGV0IHJlc3VsdCA9IG5ldyBNYXRyaXgobSAqIHAsIG4gKiBxKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IG07IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBuOyBqKyspIHtcbiAgICAgICAgZm9yIChsZXQgayA9IDA7IGsgPCBwOyBrKyspIHtcbiAgICAgICAgICBmb3IgKGxldCBsID0gMDsgbCA8IHE7IGwrKykge1xuICAgICAgICAgICAgcmVzdWx0LnNldChwICogaSArIGssIHEgKiBqICsgbCwgdGhpcy5nZXQoaSwgaikgKiBvdGhlci5nZXQoaywgbCkpO1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gcmVzdWx0O1xuICB9XG5cbiAga3JvbmVja2VyU3VtKG90aGVyKSB7XG4gICAgb3RoZXIgPSBNYXRyaXguY2hlY2tNYXRyaXgob3RoZXIpO1xuICAgIGlmICghdGhpcy5pc1NxdWFyZSgpIHx8ICFvdGhlci5pc1NxdWFyZSgpKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoJ0tyb25lY2tlciBTdW0gbmVlZHMgdHdvIFNxdWFyZSBNYXRyaWNlcycpO1xuICAgIH1cbiAgICBsZXQgbSA9IHRoaXMucm93cztcbiAgICBsZXQgbiA9IG90aGVyLnJvd3M7XG4gICAgbGV0IEF4SSA9IHRoaXMua3JvbmVja2VyUHJvZHVjdChNYXRyaXguZXllKG4sIG4pKTtcbiAgICBsZXQgSXhCID0gTWF0cml4LmV5ZShtLCBtKS5rcm9uZWNrZXJQcm9kdWN0KG90aGVyKTtcbiAgICByZXR1cm4gQXhJLmFkZChJeEIpO1xuICB9XG5cbiAgdHJhbnNwb3NlKCkge1xuICAgIGxldCByZXN1bHQgPSBuZXcgTWF0cml4KHRoaXMuY29sdW1ucywgdGhpcy5yb3dzKTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IHRoaXMuY29sdW1uczsgaisrKSB7XG4gICAgICAgIHJlc3VsdC5zZXQoaiwgaSwgdGhpcy5nZXQoaSwgaikpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gcmVzdWx0O1xuICB9XG5cbiAgc29ydFJvd3MoY29tcGFyZUZ1bmN0aW9uID0gY29tcGFyZU51bWJlcnMpIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICB0aGlzLnNldFJvdyhpLCB0aGlzLmdldFJvdyhpKS5zb3J0KGNvbXBhcmVGdW5jdGlvbikpO1xuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIHNvcnRDb2x1bW5zKGNvbXBhcmVGdW5jdGlvbiA9IGNvbXBhcmVOdW1iZXJzKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLmNvbHVtbnM7IGkrKykge1xuICAgICAgdGhpcy5zZXRDb2x1bW4oaSwgdGhpcy5nZXRDb2x1bW4oaSkuc29ydChjb21wYXJlRnVuY3Rpb24pKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBzdWJNYXRyaXgoc3RhcnRSb3csIGVuZFJvdywgc3RhcnRDb2x1bW4sIGVuZENvbHVtbikge1xuICAgIGNoZWNrUmFuZ2UodGhpcywgc3RhcnRSb3csIGVuZFJvdywgc3RhcnRDb2x1bW4sIGVuZENvbHVtbik7XG4gICAgbGV0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgoXG4gICAgICBlbmRSb3cgLSBzdGFydFJvdyArIDEsXG4gICAgICBlbmRDb2x1bW4gLSBzdGFydENvbHVtbiArIDEsXG4gICAgKTtcbiAgICBmb3IgKGxldCBpID0gc3RhcnRSb3c7IGkgPD0gZW5kUm93OyBpKyspIHtcbiAgICAgIGZvciAobGV0IGogPSBzdGFydENvbHVtbjsgaiA8PSBlbmRDb2x1bW47IGorKykge1xuICAgICAgICBuZXdNYXRyaXguc2V0KGkgLSBzdGFydFJvdywgaiAtIHN0YXJ0Q29sdW1uLCB0aGlzLmdldChpLCBqKSk7XG4gICAgICB9XG4gICAgfVxuICAgIHJldHVybiBuZXdNYXRyaXg7XG4gIH1cblxuICBzdWJNYXRyaXhSb3coaW5kaWNlcywgc3RhcnRDb2x1bW4sIGVuZENvbHVtbikge1xuICAgIGlmIChzdGFydENvbHVtbiA9PT0gdW5kZWZpbmVkKSBzdGFydENvbHVtbiA9IDA7XG4gICAgaWYgKGVuZENvbHVtbiA9PT0gdW5kZWZpbmVkKSBlbmRDb2x1bW4gPSB0aGlzLmNvbHVtbnMgLSAxO1xuICAgIGlmIChcbiAgICAgIHN0YXJ0Q29sdW1uID4gZW5kQ29sdW1uIHx8XG4gICAgICBzdGFydENvbHVtbiA8IDAgfHxcbiAgICAgIHN0YXJ0Q29sdW1uID49IHRoaXMuY29sdW1ucyB8fFxuICAgICAgZW5kQ29sdW1uIDwgMCB8fFxuICAgICAgZW5kQ29sdW1uID49IHRoaXMuY29sdW1uc1xuICAgICkge1xuICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ0FyZ3VtZW50IG91dCBvZiByYW5nZScpO1xuICAgIH1cblxuICAgIGxldCBuZXdNYXRyaXggPSBuZXcgTWF0cml4KGluZGljZXMubGVuZ3RoLCBlbmRDb2x1bW4gLSBzdGFydENvbHVtbiArIDEpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgaW5kaWNlcy5sZW5ndGg7IGkrKykge1xuICAgICAgZm9yIChsZXQgaiA9IHN0YXJ0Q29sdW1uOyBqIDw9IGVuZENvbHVtbjsgaisrKSB7XG4gICAgICAgIGlmIChpbmRpY2VzW2ldIDwgMCB8fCBpbmRpY2VzW2ldID49IHRoaXMucm93cykge1xuICAgICAgICAgIHRocm93IG5ldyBSYW5nZUVycm9yKGBSb3cgaW5kZXggb3V0IG9mIHJhbmdlOiAke2luZGljZXNbaV19YCk7XG4gICAgICAgIH1cbiAgICAgICAgbmV3TWF0cml4LnNldChpLCBqIC0gc3RhcnRDb2x1bW4sIHRoaXMuZ2V0KGluZGljZXNbaV0sIGopKTtcbiAgICAgIH1cbiAgICB9XG4gICAgcmV0dXJuIG5ld01hdHJpeDtcbiAgfVxuXG4gIHN1Yk1hdHJpeENvbHVtbihpbmRpY2VzLCBzdGFydFJvdywgZW5kUm93KSB7XG4gICAgaWYgKHN0YXJ0Um93ID09PSB1bmRlZmluZWQpIHN0YXJ0Um93ID0gMDtcbiAgICBpZiAoZW5kUm93ID09PSB1bmRlZmluZWQpIGVuZFJvdyA9IHRoaXMucm93cyAtIDE7XG4gICAgaWYgKFxuICAgICAgc3RhcnRSb3cgPiBlbmRSb3cgfHxcbiAgICAgIHN0YXJ0Um93IDwgMCB8fFxuICAgICAgc3RhcnRSb3cgPj0gdGhpcy5yb3dzIHx8XG4gICAgICBlbmRSb3cgPCAwIHx8XG4gICAgICBlbmRSb3cgPj0gdGhpcy5yb3dzXG4gICAgKSB7XG4gICAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcignQXJndW1lbnQgb3V0IG9mIHJhbmdlJyk7XG4gICAgfVxuXG4gICAgbGV0IG5ld01hdHJpeCA9IG5ldyBNYXRyaXgoZW5kUm93IC0gc3RhcnRSb3cgKyAxLCBpbmRpY2VzLmxlbmd0aCk7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBpbmRpY2VzLmxlbmd0aDsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gc3RhcnRSb3c7IGogPD0gZW5kUm93OyBqKyspIHtcbiAgICAgICAgaWYgKGluZGljZXNbaV0gPCAwIHx8IGluZGljZXNbaV0gPj0gdGhpcy5jb2x1bW5zKSB7XG4gICAgICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoYENvbHVtbiBpbmRleCBvdXQgb2YgcmFuZ2U6ICR7aW5kaWNlc1tpXX1gKTtcbiAgICAgICAgfVxuICAgICAgICBuZXdNYXRyaXguc2V0KGogLSBzdGFydFJvdywgaSwgdGhpcy5nZXQoaiwgaW5kaWNlc1tpXSkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gbmV3TWF0cml4O1xuICB9XG5cbiAgc2V0U3ViTWF0cml4KG1hdHJpeCwgc3RhcnRSb3csIHN0YXJ0Q29sdW1uKSB7XG4gICAgbWF0cml4ID0gTWF0cml4LmNoZWNrTWF0cml4KG1hdHJpeCk7XG4gICAgaWYgKG1hdHJpeC5pc0VtcHR5KCkpIHtcbiAgICAgIHJldHVybiB0aGlzO1xuICAgIH1cbiAgICBsZXQgZW5kUm93ID0gc3RhcnRSb3cgKyBtYXRyaXgucm93cyAtIDE7XG4gICAgbGV0IGVuZENvbHVtbiA9IHN0YXJ0Q29sdW1uICsgbWF0cml4LmNvbHVtbnMgLSAxO1xuICAgIGNoZWNrUmFuZ2UodGhpcywgc3RhcnRSb3csIGVuZFJvdywgc3RhcnRDb2x1bW4sIGVuZENvbHVtbik7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgaSsrKSB7XG4gICAgICBmb3IgKGxldCBqID0gMDsgaiA8IG1hdHJpeC5jb2x1bW5zOyBqKyspIHtcbiAgICAgICAgdGhpcy5zZXQoc3RhcnRSb3cgKyBpLCBzdGFydENvbHVtbiArIGosIG1hdHJpeC5nZXQoaSwgaikpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIHNlbGVjdGlvbihyb3dJbmRpY2VzLCBjb2x1bW5JbmRpY2VzKSB7XG4gICAgY2hlY2tSb3dJbmRpY2VzKHRoaXMsIHJvd0luZGljZXMpO1xuICAgIGNoZWNrQ29sdW1uSW5kaWNlcyh0aGlzLCBjb2x1bW5JbmRpY2VzKTtcbiAgICBsZXQgbmV3TWF0cml4ID0gbmV3IE1hdHJpeChyb3dJbmRpY2VzLmxlbmd0aCwgY29sdW1uSW5kaWNlcy5sZW5ndGgpO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcm93SW5kaWNlcy5sZW5ndGg7IGkrKykge1xuICAgICAgbGV0IHJvd0luZGV4ID0gcm93SW5kaWNlc1tpXTtcbiAgICAgIGZvciAobGV0IGogPSAwOyBqIDwgY29sdW1uSW5kaWNlcy5sZW5ndGg7IGorKykge1xuICAgICAgICBsZXQgY29sdW1uSW5kZXggPSBjb2x1bW5JbmRpY2VzW2pdO1xuICAgICAgICBuZXdNYXRyaXguc2V0KGksIGosIHRoaXMuZ2V0KHJvd0luZGV4LCBjb2x1bW5JbmRleCkpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gbmV3TWF0cml4O1xuICB9XG5cbiAgdHJhY2UoKSB7XG4gICAgbGV0IG1pbiA9IE1hdGgubWluKHRoaXMucm93cywgdGhpcy5jb2x1bW5zKTtcbiAgICBsZXQgdHJhY2UgPSAwO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgbWluOyBpKyspIHtcbiAgICAgIHRyYWNlICs9IHRoaXMuZ2V0KGksIGkpO1xuICAgIH1cbiAgICByZXR1cm4gdHJhY2U7XG4gIH1cblxuICBjbG9uZSgpIHtcbiAgICBsZXQgbmV3TWF0cml4ID0gbmV3IE1hdHJpeCh0aGlzLnJvd3MsIHRoaXMuY29sdW1ucyk7XG4gICAgZm9yIChsZXQgcm93ID0gMDsgcm93IDwgdGhpcy5yb3dzOyByb3crKykge1xuICAgICAgZm9yIChsZXQgY29sdW1uID0gMDsgY29sdW1uIDwgdGhpcy5jb2x1bW5zOyBjb2x1bW4rKykge1xuICAgICAgICBuZXdNYXRyaXguc2V0KHJvdywgY29sdW1uLCB0aGlzLmdldChyb3csIGNvbHVtbikpO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gbmV3TWF0cml4O1xuICB9XG5cbiAgc3VtKGJ5KSB7XG4gICAgc3dpdGNoIChieSkge1xuICAgICAgY2FzZSAncm93JzpcbiAgICAgICAgcmV0dXJuIHN1bUJ5Um93KHRoaXMpO1xuICAgICAgY2FzZSAnY29sdW1uJzpcbiAgICAgICAgcmV0dXJuIHN1bUJ5Q29sdW1uKHRoaXMpO1xuICAgICAgY2FzZSB1bmRlZmluZWQ6XG4gICAgICAgIHJldHVybiBzdW1BbGwodGhpcyk7XG4gICAgICBkZWZhdWx0OlxuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoYGludmFsaWQgb3B0aW9uOiAke2J5fWApO1xuICAgIH1cbiAgfVxuXG4gIHByb2R1Y3QoYnkpIHtcbiAgICBzd2l0Y2ggKGJ5KSB7XG4gICAgICBjYXNlICdyb3cnOlxuICAgICAgICByZXR1cm4gcHJvZHVjdEJ5Um93KHRoaXMpO1xuICAgICAgY2FzZSAnY29sdW1uJzpcbiAgICAgICAgcmV0dXJuIHByb2R1Y3RCeUNvbHVtbih0aGlzKTtcbiAgICAgIGNhc2UgdW5kZWZpbmVkOlxuICAgICAgICByZXR1cm4gcHJvZHVjdEFsbCh0aGlzKTtcbiAgICAgIGRlZmF1bHQ6XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihgaW52YWxpZCBvcHRpb246ICR7Ynl9YCk7XG4gICAgfVxuICB9XG5cbiAgbWVhbihieSkge1xuICAgIGNvbnN0IHN1bSA9IHRoaXMuc3VtKGJ5KTtcbiAgICBzd2l0Y2ggKGJ5KSB7XG4gICAgICBjYXNlICdyb3cnOiB7XG4gICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5yb3dzOyBpKyspIHtcbiAgICAgICAgICBzdW1baV0gLz0gdGhpcy5jb2x1bW5zO1xuICAgICAgICB9XG4gICAgICAgIHJldHVybiBzdW07XG4gICAgICB9XG4gICAgICBjYXNlICdjb2x1bW4nOiB7XG4gICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5jb2x1bW5zOyBpKyspIHtcbiAgICAgICAgICBzdW1baV0gLz0gdGhpcy5yb3dzO1xuICAgICAgICB9XG4gICAgICAgIHJldHVybiBzdW07XG4gICAgICB9XG4gICAgICBjYXNlIHVuZGVmaW5lZDpcbiAgICAgICAgcmV0dXJuIHN1bSAvIHRoaXMuc2l6ZTtcbiAgICAgIGRlZmF1bHQ6XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihgaW52YWxpZCBvcHRpb246ICR7Ynl9YCk7XG4gICAgfVxuICB9XG5cbiAgdmFyaWFuY2UoYnksIG9wdGlvbnMgPSB7fSkge1xuICAgIGlmICh0eXBlb2YgYnkgPT09ICdvYmplY3QnKSB7XG4gICAgICBvcHRpb25zID0gYnk7XG4gICAgICBieSA9IHVuZGVmaW5lZDtcbiAgICB9XG4gICAgaWYgKHR5cGVvZiBvcHRpb25zICE9PSAnb2JqZWN0Jykge1xuICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignb3B0aW9ucyBtdXN0IGJlIGFuIG9iamVjdCcpO1xuICAgIH1cbiAgICBjb25zdCB7IHVuYmlhc2VkID0gdHJ1ZSwgbWVhbiA9IHRoaXMubWVhbihieSkgfSA9IG9wdGlvbnM7XG4gICAgaWYgKHR5cGVvZiB1bmJpYXNlZCAhPT0gJ2Jvb2xlYW4nKSB7XG4gICAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCd1bmJpYXNlZCBtdXN0IGJlIGEgYm9vbGVhbicpO1xuICAgIH1cbiAgICBzd2l0Y2ggKGJ5KSB7XG4gICAgICBjYXNlICdyb3cnOiB7XG4gICAgICAgIGlmICghaXNBbnlBcnJheShtZWFuKSkge1xuICAgICAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ21lYW4gbXVzdCBiZSBhbiBhcnJheScpO1xuICAgICAgICB9XG4gICAgICAgIHJldHVybiB2YXJpYW5jZUJ5Um93KHRoaXMsIHVuYmlhc2VkLCBtZWFuKTtcbiAgICAgIH1cbiAgICAgIGNhc2UgJ2NvbHVtbic6IHtcbiAgICAgICAgaWYgKCFpc0FueUFycmF5KG1lYW4pKSB7XG4gICAgICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignbWVhbiBtdXN0IGJlIGFuIGFycmF5Jyk7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIHZhcmlhbmNlQnlDb2x1bW4odGhpcywgdW5iaWFzZWQsIG1lYW4pO1xuICAgICAgfVxuICAgICAgY2FzZSB1bmRlZmluZWQ6IHtcbiAgICAgICAgaWYgKHR5cGVvZiBtZWFuICE9PSAnbnVtYmVyJykge1xuICAgICAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ21lYW4gbXVzdCBiZSBhIG51bWJlcicpO1xuICAgICAgICB9XG4gICAgICAgIHJldHVybiB2YXJpYW5jZUFsbCh0aGlzLCB1bmJpYXNlZCwgbWVhbik7XG4gICAgICB9XG4gICAgICBkZWZhdWx0OlxuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoYGludmFsaWQgb3B0aW9uOiAke2J5fWApO1xuICAgIH1cbiAgfVxuXG4gIHN0YW5kYXJkRGV2aWF0aW9uKGJ5LCBvcHRpb25zKSB7XG4gICAgaWYgKHR5cGVvZiBieSA9PT0gJ29iamVjdCcpIHtcbiAgICAgIG9wdGlvbnMgPSBieTtcbiAgICAgIGJ5ID0gdW5kZWZpbmVkO1xuICAgIH1cbiAgICBjb25zdCB2YXJpYW5jZSA9IHRoaXMudmFyaWFuY2UoYnksIG9wdGlvbnMpO1xuICAgIGlmIChieSA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICByZXR1cm4gTWF0aC5zcXJ0KHZhcmlhbmNlKTtcbiAgICB9IGVsc2Uge1xuICAgICAgZm9yIChsZXQgaSA9IDA7IGkgPCB2YXJpYW5jZS5sZW5ndGg7IGkrKykge1xuICAgICAgICB2YXJpYW5jZVtpXSA9IE1hdGguc3FydCh2YXJpYW5jZVtpXSk7XG4gICAgICB9XG4gICAgICByZXR1cm4gdmFyaWFuY2U7XG4gICAgfVxuICB9XG5cbiAgY2VudGVyKGJ5LCBvcHRpb25zID0ge30pIHtcbiAgICBpZiAodHlwZW9mIGJ5ID09PSAnb2JqZWN0Jykge1xuICAgICAgb3B0aW9ucyA9IGJ5O1xuICAgICAgYnkgPSB1bmRlZmluZWQ7XG4gICAgfVxuICAgIGlmICh0eXBlb2Ygb3B0aW9ucyAhPT0gJ29iamVjdCcpIHtcbiAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ29wdGlvbnMgbXVzdCBiZSBhbiBvYmplY3QnKTtcbiAgICB9XG4gICAgY29uc3QgeyBjZW50ZXIgPSB0aGlzLm1lYW4oYnkpIH0gPSBvcHRpb25zO1xuICAgIHN3aXRjaCAoYnkpIHtcbiAgICAgIGNhc2UgJ3Jvdyc6IHtcbiAgICAgICAgaWYgKCFpc0FueUFycmF5KGNlbnRlcikpIHtcbiAgICAgICAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdjZW50ZXIgbXVzdCBiZSBhbiBhcnJheScpO1xuICAgICAgICB9XG4gICAgICAgIGNlbnRlckJ5Um93KHRoaXMsIGNlbnRlcik7XG4gICAgICAgIHJldHVybiB0aGlzO1xuICAgICAgfVxuICAgICAgY2FzZSAnY29sdW1uJzoge1xuICAgICAgICBpZiAoIWlzQW55QXJyYXkoY2VudGVyKSkge1xuICAgICAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ2NlbnRlciBtdXN0IGJlIGFuIGFycmF5Jyk7XG4gICAgICAgIH1cbiAgICAgICAgY2VudGVyQnlDb2x1bW4odGhpcywgY2VudGVyKTtcbiAgICAgICAgcmV0dXJuIHRoaXM7XG4gICAgICB9XG4gICAgICBjYXNlIHVuZGVmaW5lZDoge1xuICAgICAgICBpZiAodHlwZW9mIGNlbnRlciAhPT0gJ251bWJlcicpIHtcbiAgICAgICAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdjZW50ZXIgbXVzdCBiZSBhIG51bWJlcicpO1xuICAgICAgICB9XG4gICAgICAgIGNlbnRlckFsbCh0aGlzLCBjZW50ZXIpO1xuICAgICAgICByZXR1cm4gdGhpcztcbiAgICAgIH1cbiAgICAgIGRlZmF1bHQ6XG4gICAgICAgIHRocm93IG5ldyBFcnJvcihgaW52YWxpZCBvcHRpb246ICR7Ynl9YCk7XG4gICAgfVxuICB9XG5cbiAgc2NhbGUoYnksIG9wdGlvbnMgPSB7fSkge1xuICAgIGlmICh0eXBlb2YgYnkgPT09ICdvYmplY3QnKSB7XG4gICAgICBvcHRpb25zID0gYnk7XG4gICAgICBieSA9IHVuZGVmaW5lZDtcbiAgICB9XG4gICAgaWYgKHR5cGVvZiBvcHRpb25zICE9PSAnb2JqZWN0Jykge1xuICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignb3B0aW9ucyBtdXN0IGJlIGFuIG9iamVjdCcpO1xuICAgIH1cbiAgICBsZXQgc2NhbGUgPSBvcHRpb25zLnNjYWxlO1xuICAgIHN3aXRjaCAoYnkpIHtcbiAgICAgIGNhc2UgJ3Jvdyc6IHtcbiAgICAgICAgaWYgKHNjYWxlID09PSB1bmRlZmluZWQpIHtcbiAgICAgICAgICBzY2FsZSA9IGdldFNjYWxlQnlSb3codGhpcyk7XG4gICAgICAgIH0gZWxzZSBpZiAoIWlzQW55QXJyYXkoc2NhbGUpKSB7XG4gICAgICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignc2NhbGUgbXVzdCBiZSBhbiBhcnJheScpO1xuICAgICAgICB9XG4gICAgICAgIHNjYWxlQnlSb3codGhpcywgc2NhbGUpO1xuICAgICAgICByZXR1cm4gdGhpcztcbiAgICAgIH1cbiAgICAgIGNhc2UgJ2NvbHVtbic6IHtcbiAgICAgICAgaWYgKHNjYWxlID09PSB1bmRlZmluZWQpIHtcbiAgICAgICAgICBzY2FsZSA9IGdldFNjYWxlQnlDb2x1bW4odGhpcyk7XG4gICAgICAgIH0gZWxzZSBpZiAoIWlzQW55QXJyYXkoc2NhbGUpKSB7XG4gICAgICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignc2NhbGUgbXVzdCBiZSBhbiBhcnJheScpO1xuICAgICAgICB9XG4gICAgICAgIHNjYWxlQnlDb2x1bW4odGhpcywgc2NhbGUpO1xuICAgICAgICByZXR1cm4gdGhpcztcbiAgICAgIH1cbiAgICAgIGNhc2UgdW5kZWZpbmVkOiB7XG4gICAgICAgIGlmIChzY2FsZSA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICAgICAgc2NhbGUgPSBnZXRTY2FsZUFsbCh0aGlzKTtcbiAgICAgICAgfSBlbHNlIGlmICh0eXBlb2Ygc2NhbGUgIT09ICdudW1iZXInKSB7XG4gICAgICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignc2NhbGUgbXVzdCBiZSBhIG51bWJlcicpO1xuICAgICAgICB9XG4gICAgICAgIHNjYWxlQWxsKHRoaXMsIHNjYWxlKTtcbiAgICAgICAgcmV0dXJuIHRoaXM7XG4gICAgICB9XG4gICAgICBkZWZhdWx0OlxuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoYGludmFsaWQgb3B0aW9uOiAke2J5fWApO1xuICAgIH1cbiAgfVxuXG4gIHRvU3RyaW5nKG9wdGlvbnMpIHtcbiAgICByZXR1cm4gaW5zcGVjdE1hdHJpeFdpdGhPcHRpb25zKHRoaXMsIG9wdGlvbnMpO1xuICB9XG59XG5cbkFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5rbGFzcyA9ICdNYXRyaXgnO1xuaWYgKHR5cGVvZiBTeW1ib2wgIT09ICd1bmRlZmluZWQnKSB7XG4gIEFic3RyYWN0TWF0cml4LnByb3RvdHlwZVtTeW1ib2wuZm9yKCdub2RlanMudXRpbC5pbnNwZWN0LmN1c3RvbScpXSA9XG4gICAgaW5zcGVjdE1hdHJpeDtcbn1cblxuZnVuY3Rpb24gY29tcGFyZU51bWJlcnMoYSwgYikge1xuICByZXR1cm4gYSAtIGI7XG59XG5cbmZ1bmN0aW9uIGlzQXJyYXlPZk51bWJlcnMoYXJyYXkpIHtcbiAgcmV0dXJuIGFycmF5LmV2ZXJ5KChlbGVtZW50KSA9PiB7XG4gICAgcmV0dXJuIHR5cGVvZiBlbGVtZW50ID09PSAnbnVtYmVyJztcbiAgfSk7XG59XG5cbi8vIFN5bm9ueW1zXG5BYnN0cmFjdE1hdHJpeC5yYW5kb20gPSBBYnN0cmFjdE1hdHJpeC5yYW5kO1xuQWJzdHJhY3RNYXRyaXgucmFuZG9tSW50ID0gQWJzdHJhY3RNYXRyaXgucmFuZEludDtcbkFic3RyYWN0TWF0cml4LmRpYWdvbmFsID0gQWJzdHJhY3RNYXRyaXguZGlhZztcbkFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5kaWFnb25hbCA9IEFic3RyYWN0TWF0cml4LnByb3RvdHlwZS5kaWFnO1xuQWJzdHJhY3RNYXRyaXguaWRlbnRpdHkgPSBBYnN0cmFjdE1hdHJpeC5leWU7XG5BYnN0cmFjdE1hdHJpeC5wcm90b3R5cGUubmVnYXRlID0gQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLm5lZztcbkFic3RyYWN0TWF0cml4LnByb3RvdHlwZS50ZW5zb3JQcm9kdWN0ID1cbiAgQWJzdHJhY3RNYXRyaXgucHJvdG90eXBlLmtyb25lY2tlclByb2R1Y3Q7XG5cbmV4cG9ydCBkZWZhdWx0IGNsYXNzIE1hdHJpeCBleHRlbmRzIEFic3RyYWN0TWF0cml4IHtcbiAgY29uc3RydWN0b3IoblJvd3MsIG5Db2x1bW5zKSB7XG4gICAgc3VwZXIoKTtcbiAgICBpZiAoTWF0cml4LmlzTWF0cml4KG5Sb3dzKSkge1xuICAgICAgLy8gZXNsaW50LWRpc2FibGUtbmV4dC1saW5lIG5vLWNvbnN0cnVjdG9yLXJldHVyblxuICAgICAgcmV0dXJuIG5Sb3dzLmNsb25lKCk7XG4gICAgfSBlbHNlIGlmIChOdW1iZXIuaXNJbnRlZ2VyKG5Sb3dzKSAmJiBuUm93cyA+PSAwKSB7XG4gICAgICAvLyBDcmVhdGUgYW4gZW1wdHkgbWF0cml4XG4gICAgICB0aGlzLmRhdGEgPSBbXTtcbiAgICAgIGlmIChOdW1iZXIuaXNJbnRlZ2VyKG5Db2x1bW5zKSAmJiBuQ29sdW1ucyA+PSAwKSB7XG4gICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgblJvd3M7IGkrKykge1xuICAgICAgICAgIHRoaXMuZGF0YS5wdXNoKG5ldyBGbG9hdDY0QXJyYXkobkNvbHVtbnMpKTtcbiAgICAgICAgfVxuICAgICAgfSBlbHNlIHtcbiAgICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcignbkNvbHVtbnMgbXVzdCBiZSBhIHBvc2l0aXZlIGludGVnZXInKTtcbiAgICAgIH1cbiAgICB9IGVsc2UgaWYgKGlzQW55QXJyYXkoblJvd3MpKSB7XG4gICAgICAvLyBDb3B5IHRoZSB2YWx1ZXMgZnJvbSB0aGUgMkQgYXJyYXlcbiAgICAgIGNvbnN0IGFycmF5RGF0YSA9IG5Sb3dzO1xuICAgICAgblJvd3MgPSBhcnJheURhdGEubGVuZ3RoO1xuICAgICAgbkNvbHVtbnMgPSBuUm93cyA/IGFycmF5RGF0YVswXS5sZW5ndGggOiAwO1xuICAgICAgaWYgKHR5cGVvZiBuQ29sdW1ucyAhPT0gJ251bWJlcicpIHtcbiAgICAgICAgdGhyb3cgbmV3IFR5cGVFcnJvcihcbiAgICAgICAgICAnRGF0YSBtdXN0IGJlIGEgMkQgYXJyYXkgd2l0aCBhdCBsZWFzdCBvbmUgZWxlbWVudCcsXG4gICAgICAgICk7XG4gICAgICB9XG4gICAgICB0aGlzLmRhdGEgPSBbXTtcbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgblJvd3M7IGkrKykge1xuICAgICAgICBpZiAoYXJyYXlEYXRhW2ldLmxlbmd0aCAhPT0gbkNvbHVtbnMpIHtcbiAgICAgICAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcignSW5jb25zaXN0ZW50IGFycmF5IGRpbWVuc2lvbnMnKTtcbiAgICAgICAgfVxuICAgICAgICBpZiAoIWlzQXJyYXlPZk51bWJlcnMoYXJyYXlEYXRhW2ldKSkge1xuICAgICAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ0lucHV0IGRhdGEgY29udGFpbnMgbm9uLW51bWVyaWMgdmFsdWVzJyk7XG4gICAgICAgIH1cbiAgICAgICAgdGhpcy5kYXRhLnB1c2goRmxvYXQ2NEFycmF5LmZyb20oYXJyYXlEYXRhW2ldKSk7XG4gICAgICB9XG4gICAgfSBlbHNlIHtcbiAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoXG4gICAgICAgICdGaXJzdCBhcmd1bWVudCBtdXN0IGJlIGEgcG9zaXRpdmUgbnVtYmVyIG9yIGFuIGFycmF5JyxcbiAgICAgICk7XG4gICAgfVxuICAgIHRoaXMucm93cyA9IG5Sb3dzO1xuICAgIHRoaXMuY29sdW1ucyA9IG5Db2x1bW5zO1xuICB9XG5cbiAgc2V0KHJvd0luZGV4LCBjb2x1bW5JbmRleCwgdmFsdWUpIHtcbiAgICB0aGlzLmRhdGFbcm93SW5kZXhdW2NvbHVtbkluZGV4XSA9IHZhbHVlO1xuICAgIHJldHVybiB0aGlzO1xuICB9XG5cbiAgZ2V0KHJvd0luZGV4LCBjb2x1bW5JbmRleCkge1xuICAgIHJldHVybiB0aGlzLmRhdGFbcm93SW5kZXhdW2NvbHVtbkluZGV4XTtcbiAgfVxuXG4gIHJlbW92ZVJvdyhpbmRleCkge1xuICAgIGNoZWNrUm93SW5kZXgodGhpcywgaW5kZXgpO1xuICAgIHRoaXMuZGF0YS5zcGxpY2UoaW5kZXgsIDEpO1xuICAgIHRoaXMucm93cyAtPSAxO1xuICAgIHJldHVybiB0aGlzO1xuICB9XG5cbiAgYWRkUm93KGluZGV4LCBhcnJheSkge1xuICAgIGlmIChhcnJheSA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICBhcnJheSA9IGluZGV4O1xuICAgICAgaW5kZXggPSB0aGlzLnJvd3M7XG4gICAgfVxuICAgIGNoZWNrUm93SW5kZXgodGhpcywgaW5kZXgsIHRydWUpO1xuICAgIGFycmF5ID0gRmxvYXQ2NEFycmF5LmZyb20oY2hlY2tSb3dWZWN0b3IodGhpcywgYXJyYXkpKTtcbiAgICB0aGlzLmRhdGEuc3BsaWNlKGluZGV4LCAwLCBhcnJheSk7XG4gICAgdGhpcy5yb3dzICs9IDE7XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICByZW1vdmVDb2x1bW4oaW5kZXgpIHtcbiAgICBjaGVja0NvbHVtbkluZGV4KHRoaXMsIGluZGV4KTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBjb25zdCBuZXdSb3cgPSBuZXcgRmxvYXQ2NEFycmF5KHRoaXMuY29sdW1ucyAtIDEpO1xuICAgICAgZm9yIChsZXQgaiA9IDA7IGogPCBpbmRleDsgaisrKSB7XG4gICAgICAgIG5ld1Jvd1tqXSA9IHRoaXMuZGF0YVtpXVtqXTtcbiAgICAgIH1cbiAgICAgIGZvciAobGV0IGogPSBpbmRleCArIDE7IGogPCB0aGlzLmNvbHVtbnM7IGorKykge1xuICAgICAgICBuZXdSb3dbaiAtIDFdID0gdGhpcy5kYXRhW2ldW2pdO1xuICAgICAgfVxuICAgICAgdGhpcy5kYXRhW2ldID0gbmV3Um93O1xuICAgIH1cbiAgICB0aGlzLmNvbHVtbnMgLT0gMTtcbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIGFkZENvbHVtbihpbmRleCwgYXJyYXkpIHtcbiAgICBpZiAodHlwZW9mIGFycmF5ID09PSAndW5kZWZpbmVkJykge1xuICAgICAgYXJyYXkgPSBpbmRleDtcbiAgICAgIGluZGV4ID0gdGhpcy5jb2x1bW5zO1xuICAgIH1cbiAgICBjaGVja0NvbHVtbkluZGV4KHRoaXMsIGluZGV4LCB0cnVlKTtcbiAgICBhcnJheSA9IGNoZWNrQ29sdW1uVmVjdG9yKHRoaXMsIGFycmF5KTtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMucm93czsgaSsrKSB7XG4gICAgICBjb25zdCBuZXdSb3cgPSBuZXcgRmxvYXQ2NEFycmF5KHRoaXMuY29sdW1ucyArIDEpO1xuICAgICAgbGV0IGogPSAwO1xuICAgICAgZm9yICg7IGogPCBpbmRleDsgaisrKSB7XG4gICAgICAgIG5ld1Jvd1tqXSA9IHRoaXMuZGF0YVtpXVtqXTtcbiAgICAgIH1cbiAgICAgIG5ld1Jvd1tqKytdID0gYXJyYXlbaV07XG4gICAgICBmb3IgKDsgaiA8IHRoaXMuY29sdW1ucyArIDE7IGorKykge1xuICAgICAgICBuZXdSb3dbal0gPSB0aGlzLmRhdGFbaV1baiAtIDFdO1xuICAgICAgfVxuICAgICAgdGhpcy5kYXRhW2ldID0gbmV3Um93O1xuICAgIH1cbiAgICB0aGlzLmNvbHVtbnMgKz0gMTtcbiAgICByZXR1cm4gdGhpcztcbiAgfVxufVxuXG5pbnN0YWxsTWF0aE9wZXJhdGlvbnMoQWJzdHJhY3RNYXRyaXgsIE1hdHJpeCk7XG4iLCJpbXBvcnQgU1ZEIGZyb20gJy4vZGMvc3ZkJztcbmltcG9ydCBNYXRyaXggZnJvbSAnLi9tYXRyaXgnO1xuXG5leHBvcnQgZnVuY3Rpb24gcHNldWRvSW52ZXJzZShtYXRyaXgsIHRocmVzaG9sZCA9IE51bWJlci5FUFNJTE9OKSB7XG4gIG1hdHJpeCA9IE1hdHJpeC5jaGVja01hdHJpeChtYXRyaXgpO1xuICBpZiAobWF0cml4LmlzRW1wdHkoKSkge1xuICAgIC8vIHdpdGggYSB6ZXJvIGRpbWVuc2lvbiwgdGhlIHBzZXVkby1pbnZlcnNlIGlzIHRoZSB0cmFuc3Bvc2UsIHNpbmNlIGFsbCAweG4gYW5kIG54MCBtYXRyaWNlcyBhcmUgc2luZ3VsYXJcbiAgICAvLyAoMHhuKSoobngwKSooMHhuKSA9IDB4blxuICAgIC8vIChueDApKigweG4pKihueDApID0gbngwXG4gICAgcmV0dXJuIG1hdHJpeC50cmFuc3Bvc2UoKTtcbiAgfVxuICBsZXQgc3ZkU29sdXRpb24gPSBuZXcgU1ZEKG1hdHJpeCwgeyBhdXRvVHJhbnNwb3NlOiB0cnVlIH0pO1xuXG4gIGxldCBVID0gc3ZkU29sdXRpb24ubGVmdFNpbmd1bGFyVmVjdG9ycztcbiAgbGV0IFYgPSBzdmRTb2x1dGlvbi5yaWdodFNpbmd1bGFyVmVjdG9ycztcbiAgbGV0IHMgPSBzdmRTb2x1dGlvbi5kaWFnb25hbDtcblxuICBmb3IgKGxldCBpID0gMDsgaSA8IHMubGVuZ3RoOyBpKyspIHtcbiAgICBpZiAoTWF0aC5hYnMoc1tpXSkgPiB0aHJlc2hvbGQpIHtcbiAgICAgIHNbaV0gPSAxLjAgLyBzW2ldO1xuICAgIH0gZWxzZSB7XG4gICAgICBzW2ldID0gMC4wO1xuICAgIH1cbiAgfVxuXG4gIHJldHVybiBWLm1tdWwoTWF0cml4LmRpYWcocykubW11bChVLnRyYW5zcG9zZSgpKSk7XG59XG4iLCJpbXBvcnQgeyBuZXdBcnJheSB9IGZyb20gJy4vdXRpbCc7XG5cbmV4cG9ydCBmdW5jdGlvbiBzdW1CeVJvdyhtYXRyaXgpIHtcbiAgbGV0IHN1bSA9IG5ld0FycmF5KG1hdHJpeC5yb3dzKTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgKytpKSB7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgKytqKSB7XG4gICAgICBzdW1baV0gKz0gbWF0cml4LmdldChpLCBqKTtcbiAgICB9XG4gIH1cbiAgcmV0dXJuIHN1bTtcbn1cblxuZXhwb3J0IGZ1bmN0aW9uIHN1bUJ5Q29sdW1uKG1hdHJpeCkge1xuICBsZXQgc3VtID0gbmV3QXJyYXkobWF0cml4LmNvbHVtbnMpO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IG1hdHJpeC5yb3dzOyArK2kpIHtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IG1hdHJpeC5jb2x1bW5zOyArK2opIHtcbiAgICAgIHN1bVtqXSArPSBtYXRyaXguZ2V0KGksIGopO1xuICAgIH1cbiAgfVxuICByZXR1cm4gc3VtO1xufVxuXG5leHBvcnQgZnVuY3Rpb24gc3VtQWxsKG1hdHJpeCkge1xuICBsZXQgdiA9IDA7XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbWF0cml4LnJvd3M7IGkrKykge1xuICAgIGZvciAobGV0IGogPSAwOyBqIDwgbWF0cml4LmNvbHVtbnM7IGorKykge1xuICAgICAgdiArPSBtYXRyaXguZ2V0KGksIGopO1xuICAgIH1cbiAgfVxuICByZXR1cm4gdjtcbn1cblxuZXhwb3J0IGZ1bmN0aW9uIHByb2R1Y3RCeVJvdyhtYXRyaXgpIHtcbiAgbGV0IHN1bSA9IG5ld0FycmF5KG1hdHJpeC5yb3dzLCAxKTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgKytpKSB7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgKytqKSB7XG4gICAgICBzdW1baV0gKj0gbWF0cml4LmdldChpLCBqKTtcbiAgICB9XG4gIH1cbiAgcmV0dXJuIHN1bTtcbn1cblxuZXhwb3J0IGZ1bmN0aW9uIHByb2R1Y3RCeUNvbHVtbihtYXRyaXgpIHtcbiAgbGV0IHN1bSA9IG5ld0FycmF5KG1hdHJpeC5jb2x1bW5zLCAxKTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgKytpKSB7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgKytqKSB7XG4gICAgICBzdW1bal0gKj0gbWF0cml4LmdldChpLCBqKTtcbiAgICB9XG4gIH1cbiAgcmV0dXJuIHN1bTtcbn1cblxuZXhwb3J0IGZ1bmN0aW9uIHByb2R1Y3RBbGwobWF0cml4KSB7XG4gIGxldCB2ID0gMTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgaSsrKSB7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgaisrKSB7XG4gICAgICB2ICo9IG1hdHJpeC5nZXQoaSwgaik7XG4gICAgfVxuICB9XG4gIHJldHVybiB2O1xufVxuXG5leHBvcnQgZnVuY3Rpb24gdmFyaWFuY2VCeVJvdyhtYXRyaXgsIHVuYmlhc2VkLCBtZWFuKSB7XG4gIGNvbnN0IHJvd3MgPSBtYXRyaXgucm93cztcbiAgY29uc3QgY29scyA9IG1hdHJpeC5jb2x1bW5zO1xuICBjb25zdCB2YXJpYW5jZSA9IFtdO1xuXG4gIGZvciAobGV0IGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgbGV0IHN1bTEgPSAwO1xuICAgIGxldCBzdW0yID0gMDtcbiAgICBsZXQgeCA9IDA7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBjb2xzOyBqKyspIHtcbiAgICAgIHggPSBtYXRyaXguZ2V0KGksIGopIC0gbWVhbltpXTtcbiAgICAgIHN1bTEgKz0geDtcbiAgICAgIHN1bTIgKz0geCAqIHg7XG4gICAgfVxuICAgIGlmICh1bmJpYXNlZCkge1xuICAgICAgdmFyaWFuY2UucHVzaCgoc3VtMiAtIChzdW0xICogc3VtMSkgLyBjb2xzKSAvIChjb2xzIC0gMSkpO1xuICAgIH0gZWxzZSB7XG4gICAgICB2YXJpYW5jZS5wdXNoKChzdW0yIC0gKHN1bTEgKiBzdW0xKSAvIGNvbHMpIC8gY29scyk7XG4gICAgfVxuICB9XG4gIHJldHVybiB2YXJpYW5jZTtcbn1cblxuZXhwb3J0IGZ1bmN0aW9uIHZhcmlhbmNlQnlDb2x1bW4obWF0cml4LCB1bmJpYXNlZCwgbWVhbikge1xuICBjb25zdCByb3dzID0gbWF0cml4LnJvd3M7XG4gIGNvbnN0IGNvbHMgPSBtYXRyaXguY29sdW1ucztcbiAgY29uc3QgdmFyaWFuY2UgPSBbXTtcblxuICBmb3IgKGxldCBqID0gMDsgaiA8IGNvbHM7IGorKykge1xuICAgIGxldCBzdW0xID0gMDtcbiAgICBsZXQgc3VtMiA9IDA7XG4gICAgbGV0IHggPSAwO1xuICAgIGZvciAobGV0IGkgPSAwOyBpIDwgcm93czsgaSsrKSB7XG4gICAgICB4ID0gbWF0cml4LmdldChpLCBqKSAtIG1lYW5bal07XG4gICAgICBzdW0xICs9IHg7XG4gICAgICBzdW0yICs9IHggKiB4O1xuICAgIH1cbiAgICBpZiAodW5iaWFzZWQpIHtcbiAgICAgIHZhcmlhbmNlLnB1c2goKHN1bTIgLSAoc3VtMSAqIHN1bTEpIC8gcm93cykgLyAocm93cyAtIDEpKTtcbiAgICB9IGVsc2Uge1xuICAgICAgdmFyaWFuY2UucHVzaCgoc3VtMiAtIChzdW0xICogc3VtMSkgLyByb3dzKSAvIHJvd3MpO1xuICAgIH1cbiAgfVxuICByZXR1cm4gdmFyaWFuY2U7XG59XG5cbmV4cG9ydCBmdW5jdGlvbiB2YXJpYW5jZUFsbChtYXRyaXgsIHVuYmlhc2VkLCBtZWFuKSB7XG4gIGNvbnN0IHJvd3MgPSBtYXRyaXgucm93cztcbiAgY29uc3QgY29scyA9IG1hdHJpeC5jb2x1bW5zO1xuICBjb25zdCBzaXplID0gcm93cyAqIGNvbHM7XG5cbiAgbGV0IHN1bTEgPSAwO1xuICBsZXQgc3VtMiA9IDA7XG4gIGxldCB4ID0gMDtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCByb3dzOyBpKyspIHtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IGNvbHM7IGorKykge1xuICAgICAgeCA9IG1hdHJpeC5nZXQoaSwgaikgLSBtZWFuO1xuICAgICAgc3VtMSArPSB4O1xuICAgICAgc3VtMiArPSB4ICogeDtcbiAgICB9XG4gIH1cbiAgaWYgKHVuYmlhc2VkKSB7XG4gICAgcmV0dXJuIChzdW0yIC0gKHN1bTEgKiBzdW0xKSAvIHNpemUpIC8gKHNpemUgLSAxKTtcbiAgfSBlbHNlIHtcbiAgICByZXR1cm4gKHN1bTIgLSAoc3VtMSAqIHN1bTEpIC8gc2l6ZSkgLyBzaXplO1xuICB9XG59XG5cbmV4cG9ydCBmdW5jdGlvbiBjZW50ZXJCeVJvdyhtYXRyaXgsIG1lYW4pIHtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgaSsrKSB7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgaisrKSB7XG4gICAgICBtYXRyaXguc2V0KGksIGosIG1hdHJpeC5nZXQoaSwgaikgLSBtZWFuW2ldKTtcbiAgICB9XG4gIH1cbn1cblxuZXhwb3J0IGZ1bmN0aW9uIGNlbnRlckJ5Q29sdW1uKG1hdHJpeCwgbWVhbikge1xuICBmb3IgKGxldCBpID0gMDsgaSA8IG1hdHJpeC5yb3dzOyBpKyspIHtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IG1hdHJpeC5jb2x1bW5zOyBqKyspIHtcbiAgICAgIG1hdHJpeC5zZXQoaSwgaiwgbWF0cml4LmdldChpLCBqKSAtIG1lYW5bal0pO1xuICAgIH1cbiAgfVxufVxuXG5leHBvcnQgZnVuY3Rpb24gY2VudGVyQWxsKG1hdHJpeCwgbWVhbikge1xuICBmb3IgKGxldCBpID0gMDsgaSA8IG1hdHJpeC5yb3dzOyBpKyspIHtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IG1hdHJpeC5jb2x1bW5zOyBqKyspIHtcbiAgICAgIG1hdHJpeC5zZXQoaSwgaiwgbWF0cml4LmdldChpLCBqKSAtIG1lYW4pO1xuICAgIH1cbiAgfVxufVxuXG5leHBvcnQgZnVuY3Rpb24gZ2V0U2NhbGVCeVJvdyhtYXRyaXgpIHtcbiAgY29uc3Qgc2NhbGUgPSBbXTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgaSsrKSB7XG4gICAgbGV0IHN1bSA9IDA7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgaisrKSB7XG4gICAgICBzdW0gKz0gTWF0aC5wb3cobWF0cml4LmdldChpLCBqKSwgMikgLyAobWF0cml4LmNvbHVtbnMgLSAxKTtcbiAgICB9XG4gICAgc2NhbGUucHVzaChNYXRoLnNxcnQoc3VtKSk7XG4gIH1cbiAgcmV0dXJuIHNjYWxlO1xufVxuXG5leHBvcnQgZnVuY3Rpb24gc2NhbGVCeVJvdyhtYXRyaXgsIHNjYWxlKSB7XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbWF0cml4LnJvd3M7IGkrKykge1xuICAgIGZvciAobGV0IGogPSAwOyBqIDwgbWF0cml4LmNvbHVtbnM7IGorKykge1xuICAgICAgbWF0cml4LnNldChpLCBqLCBtYXRyaXguZ2V0KGksIGopIC8gc2NhbGVbaV0pO1xuICAgIH1cbiAgfVxufVxuXG5leHBvcnQgZnVuY3Rpb24gZ2V0U2NhbGVCeUNvbHVtbihtYXRyaXgpIHtcbiAgY29uc3Qgc2NhbGUgPSBbXTtcbiAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgaisrKSB7XG4gICAgbGV0IHN1bSA9IDA7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgaSsrKSB7XG4gICAgICBzdW0gKz0gTWF0aC5wb3cobWF0cml4LmdldChpLCBqKSwgMikgLyAobWF0cml4LnJvd3MgLSAxKTtcbiAgICB9XG4gICAgc2NhbGUucHVzaChNYXRoLnNxcnQoc3VtKSk7XG4gIH1cbiAgcmV0dXJuIHNjYWxlO1xufVxuXG5leHBvcnQgZnVuY3Rpb24gc2NhbGVCeUNvbHVtbihtYXRyaXgsIHNjYWxlKSB7XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgbWF0cml4LnJvd3M7IGkrKykge1xuICAgIGZvciAobGV0IGogPSAwOyBqIDwgbWF0cml4LmNvbHVtbnM7IGorKykge1xuICAgICAgbWF0cml4LnNldChpLCBqLCBtYXRyaXguZ2V0KGksIGopIC8gc2NhbGVbal0pO1xuICAgIH1cbiAgfVxufVxuXG5leHBvcnQgZnVuY3Rpb24gZ2V0U2NhbGVBbGwobWF0cml4KSB7XG4gIGNvbnN0IGRpdmlkZXIgPSBtYXRyaXguc2l6ZSAtIDE7XG4gIGxldCBzdW0gPSAwO1xuICBmb3IgKGxldCBqID0gMDsgaiA8IG1hdHJpeC5jb2x1bW5zOyBqKyspIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IG1hdHJpeC5yb3dzOyBpKyspIHtcbiAgICAgIHN1bSArPSBNYXRoLnBvdyhtYXRyaXguZ2V0KGksIGopLCAyKSAvIGRpdmlkZXI7XG4gICAgfVxuICB9XG4gIHJldHVybiBNYXRoLnNxcnQoc3VtKTtcbn1cblxuZXhwb3J0IGZ1bmN0aW9uIHNjYWxlQWxsKG1hdHJpeCwgc2NhbGUpIHtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBtYXRyaXgucm93czsgaSsrKSB7XG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBtYXRyaXguY29sdW1uczsgaisrKSB7XG4gICAgICBtYXRyaXguc2V0KGksIGosIG1hdHJpeC5nZXQoaSwgaikgLyBzY2FsZSk7XG4gICAgfVxuICB9XG59XG4iLCJpbXBvcnQgeyBpc0FueUFycmF5IH0gZnJvbSAnaXMtYW55LWFycmF5JztcblxuLyoqXG4gKiBAcHJpdmF0ZVxuICogQ2hlY2sgdGhhdCBhIHJvdyBpbmRleCBpcyBub3Qgb3V0IG9mIGJvdW5kc1xuICogQHBhcmFtIHtNYXRyaXh9IG1hdHJpeFxuICogQHBhcmFtIHtudW1iZXJ9IGluZGV4XG4gKiBAcGFyYW0ge2Jvb2xlYW59IFtvdXRlcl1cbiAqL1xuZXhwb3J0IGZ1bmN0aW9uIGNoZWNrUm93SW5kZXgobWF0cml4LCBpbmRleCwgb3V0ZXIpIHtcbiAgbGV0IG1heCA9IG91dGVyID8gbWF0cml4LnJvd3MgOiBtYXRyaXgucm93cyAtIDE7XG4gIGlmIChpbmRleCA8IDAgfHwgaW5kZXggPiBtYXgpIHtcbiAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcignUm93IGluZGV4IG91dCBvZiByYW5nZScpO1xuICB9XG59XG5cbi8qKlxuICogQHByaXZhdGVcbiAqIENoZWNrIHRoYXQgYSBjb2x1bW4gaW5kZXggaXMgbm90IG91dCBvZiBib3VuZHNcbiAqIEBwYXJhbSB7TWF0cml4fSBtYXRyaXhcbiAqIEBwYXJhbSB7bnVtYmVyfSBpbmRleFxuICogQHBhcmFtIHtib29sZWFufSBbb3V0ZXJdXG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBjaGVja0NvbHVtbkluZGV4KG1hdHJpeCwgaW5kZXgsIG91dGVyKSB7XG4gIGxldCBtYXggPSBvdXRlciA/IG1hdHJpeC5jb2x1bW5zIDogbWF0cml4LmNvbHVtbnMgLSAxO1xuICBpZiAoaW5kZXggPCAwIHx8IGluZGV4ID4gbWF4KSB7XG4gICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ0NvbHVtbiBpbmRleCBvdXQgb2YgcmFuZ2UnKTtcbiAgfVxufVxuXG4vKipcbiAqIEBwcml2YXRlXG4gKiBDaGVjayB0aGF0IHRoZSBwcm92aWRlZCB2ZWN0b3IgaXMgYW4gYXJyYXkgd2l0aCB0aGUgcmlnaHQgbGVuZ3RoXG4gKiBAcGFyYW0ge01hdHJpeH0gbWF0cml4XG4gKiBAcGFyYW0ge0FycmF5fE1hdHJpeH0gdmVjdG9yXG4gKiBAcmV0dXJuIHtBcnJheX1cbiAqIEB0aHJvd3Mge1JhbmdlRXJyb3J9XG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBjaGVja1Jvd1ZlY3RvcihtYXRyaXgsIHZlY3Rvcikge1xuICBpZiAodmVjdG9yLnRvMURBcnJheSkge1xuICAgIHZlY3RvciA9IHZlY3Rvci50bzFEQXJyYXkoKTtcbiAgfVxuICBpZiAodmVjdG9yLmxlbmd0aCAhPT0gbWF0cml4LmNvbHVtbnMpIHtcbiAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcihcbiAgICAgICd2ZWN0b3Igc2l6ZSBtdXN0IGJlIHRoZSBzYW1lIGFzIHRoZSBudW1iZXIgb2YgY29sdW1ucycsXG4gICAgKTtcbiAgfVxuICByZXR1cm4gdmVjdG9yO1xufVxuXG4vKipcbiAqIEBwcml2YXRlXG4gKiBDaGVjayB0aGF0IHRoZSBwcm92aWRlZCB2ZWN0b3IgaXMgYW4gYXJyYXkgd2l0aCB0aGUgcmlnaHQgbGVuZ3RoXG4gKiBAcGFyYW0ge01hdHJpeH0gbWF0cml4XG4gKiBAcGFyYW0ge0FycmF5fE1hdHJpeH0gdmVjdG9yXG4gKiBAcmV0dXJuIHtBcnJheX1cbiAqIEB0aHJvd3Mge1JhbmdlRXJyb3J9XG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBjaGVja0NvbHVtblZlY3RvcihtYXRyaXgsIHZlY3Rvcikge1xuICBpZiAodmVjdG9yLnRvMURBcnJheSkge1xuICAgIHZlY3RvciA9IHZlY3Rvci50bzFEQXJyYXkoKTtcbiAgfVxuICBpZiAodmVjdG9yLmxlbmd0aCAhPT0gbWF0cml4LnJvd3MpIHtcbiAgICB0aHJvdyBuZXcgUmFuZ2VFcnJvcigndmVjdG9yIHNpemUgbXVzdCBiZSB0aGUgc2FtZSBhcyB0aGUgbnVtYmVyIG9mIHJvd3MnKTtcbiAgfVxuICByZXR1cm4gdmVjdG9yO1xufVxuXG5leHBvcnQgZnVuY3Rpb24gY2hlY2tSb3dJbmRpY2VzKG1hdHJpeCwgcm93SW5kaWNlcykge1xuICBpZiAoIWlzQW55QXJyYXkocm93SW5kaWNlcykpIHtcbiAgICB0aHJvdyBuZXcgVHlwZUVycm9yKCdyb3cgaW5kaWNlcyBtdXN0IGJlIGFuIGFycmF5Jyk7XG4gIH1cblxuICBmb3IgKGxldCBpID0gMDsgaSA8IHJvd0luZGljZXMubGVuZ3RoOyBpKyspIHtcbiAgICBpZiAocm93SW5kaWNlc1tpXSA8IDAgfHwgcm93SW5kaWNlc1tpXSA+PSBtYXRyaXgucm93cykge1xuICAgICAgdGhyb3cgbmV3IFJhbmdlRXJyb3IoJ3JvdyBpbmRpY2VzIGFyZSBvdXQgb2YgcmFuZ2UnKTtcbiAgICB9XG4gIH1cbn1cblxuZXhwb3J0IGZ1bmN0aW9uIGNoZWNrQ29sdW1uSW5kaWNlcyhtYXRyaXgsIGNvbHVtbkluZGljZXMpIHtcbiAgaWYgKCFpc0FueUFycmF5KGNvbHVtbkluZGljZXMpKSB7XG4gICAgdGhyb3cgbmV3IFR5cGVFcnJvcignY29sdW1uIGluZGljZXMgbXVzdCBiZSBhbiBhcnJheScpO1xuICB9XG5cbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBjb2x1bW5JbmRpY2VzLmxlbmd0aDsgaSsrKSB7XG4gICAgaWYgKGNvbHVtbkluZGljZXNbaV0gPCAwIHx8IGNvbHVtbkluZGljZXNbaV0gPj0gbWF0cml4LmNvbHVtbnMpIHtcbiAgICAgIHRocm93IG5ldyBSYW5nZUVycm9yKCdjb2x1bW4gaW5kaWNlcyBhcmUgb3V0IG9mIHJhbmdlJyk7XG4gICAgfVxuICB9XG59XG5cbmV4cG9ydCBmdW5jdGlvbiBjaGVja1JhbmdlKG1hdHJpeCwgc3RhcnRSb3csIGVuZFJvdywgc3RhcnRDb2x1bW4sIGVuZENvbHVtbikge1xuICBpZiAoYXJndW1lbnRzLmxlbmd0aCAhPT0gNSkge1xuICAgIHRocm93IG5ldyBSYW5nZUVycm9yKCdleHBlY3RlZCA0IGFyZ3VtZW50cycpO1xuICB9XG4gIGNoZWNrTnVtYmVyKCdzdGFydFJvdycsIHN0YXJ0Um93KTtcbiAgY2hlY2tOdW1iZXIoJ2VuZFJvdycsIGVuZFJvdyk7XG4gIGNoZWNrTnVtYmVyKCdzdGFydENvbHVtbicsIHN0YXJ0Q29sdW1uKTtcbiAgY2hlY2tOdW1iZXIoJ2VuZENvbHVtbicsIGVuZENvbHVtbik7XG4gIGlmIChcbiAgICBzdGFydFJvdyA+IGVuZFJvdyB8fFxuICAgIHN0YXJ0Q29sdW1uID4gZW5kQ29sdW1uIHx8XG4gICAgc3RhcnRSb3cgPCAwIHx8XG4gICAgc3RhcnRSb3cgPj0gbWF0cml4LnJvd3MgfHxcbiAgICBlbmRSb3cgPCAwIHx8XG4gICAgZW5kUm93ID49IG1hdHJpeC5yb3dzIHx8XG4gICAgc3RhcnRDb2x1bW4gPCAwIHx8XG4gICAgc3RhcnRDb2x1bW4gPj0gbWF0cml4LmNvbHVtbnMgfHxcbiAgICBlbmRDb2x1bW4gPCAwIHx8XG4gICAgZW5kQ29sdW1uID49IG1hdHJpeC5jb2x1bW5zXG4gICkge1xuICAgIHRocm93IG5ldyBSYW5nZUVycm9yKCdTdWJtYXRyaXggaW5kaWNlcyBhcmUgb3V0IG9mIHJhbmdlJyk7XG4gIH1cbn1cblxuZXhwb3J0IGZ1bmN0aW9uIG5ld0FycmF5KGxlbmd0aCwgdmFsdWUgPSAwKSB7XG4gIGxldCBhcnJheSA9IFtdO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGxlbmd0aDsgaSsrKSB7XG4gICAgYXJyYXkucHVzaCh2YWx1ZSk7XG4gIH1cbiAgcmV0dXJuIGFycmF5O1xufVxuXG5mdW5jdGlvbiBjaGVja051bWJlcihuYW1lLCB2YWx1ZSkge1xuICBpZiAodHlwZW9mIHZhbHVlICE9PSAnbnVtYmVyJykge1xuICAgIHRocm93IG5ldyBUeXBlRXJyb3IoYCR7bmFtZX0gbXVzdCBiZSBhIG51bWJlcmApO1xuICB9XG59XG5cbmV4cG9ydCBmdW5jdGlvbiBjaGVja05vbkVtcHR5KG1hdHJpeCkge1xuICBpZiAobWF0cml4LmlzRW1wdHkoKSkge1xuICAgIHRocm93IG5ldyBFcnJvcignRW1wdHkgbWF0cml4IGhhcyBubyBlbGVtZW50cyB0byBpbmRleCcpO1xuICB9XG59XG4iLCJpbXBvcnQgeyBBYnN0cmFjdE1hdHJpeCB9IGZyb20gJy4uL21hdHJpeCc7XG5cbmV4cG9ydCBkZWZhdWx0IGNsYXNzIEJhc2VWaWV3IGV4dGVuZHMgQWJzdHJhY3RNYXRyaXgge1xuICBjb25zdHJ1Y3RvcihtYXRyaXgsIHJvd3MsIGNvbHVtbnMpIHtcbiAgICBzdXBlcigpO1xuICAgIHRoaXMubWF0cml4ID0gbWF0cml4O1xuICAgIHRoaXMucm93cyA9IHJvd3M7XG4gICAgdGhpcy5jb2x1bW5zID0gY29sdW1ucztcbiAgfVxufVxuIiwiaW1wb3J0IHsgY2hlY2tDb2x1bW5JbmRleCB9IGZyb20gJy4uL3V0aWwnO1xuXG5pbXBvcnQgQmFzZVZpZXcgZnJvbSAnLi9iYXNlJztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgTWF0cml4Q29sdW1uVmlldyBleHRlbmRzIEJhc2VWaWV3IHtcbiAgY29uc3RydWN0b3IobWF0cml4LCBjb2x1bW4pIHtcbiAgICBjaGVja0NvbHVtbkluZGV4KG1hdHJpeCwgY29sdW1uKTtcbiAgICBzdXBlcihtYXRyaXgsIG1hdHJpeC5yb3dzLCAxKTtcbiAgICB0aGlzLmNvbHVtbiA9IGNvbHVtbjtcbiAgfVxuXG4gIHNldChyb3dJbmRleCwgY29sdW1uSW5kZXgsIHZhbHVlKSB7XG4gICAgdGhpcy5tYXRyaXguc2V0KHJvd0luZGV4LCB0aGlzLmNvbHVtbiwgdmFsdWUpO1xuICAgIHJldHVybiB0aGlzO1xuICB9XG5cbiAgZ2V0KHJvd0luZGV4KSB7XG4gICAgcmV0dXJuIHRoaXMubWF0cml4LmdldChyb3dJbmRleCwgdGhpcy5jb2x1bW4pO1xuICB9XG59XG4iLCJpbXBvcnQgeyBjaGVja0NvbHVtbkluZGljZXMgfSBmcm9tICcuLi91dGlsJztcblxuaW1wb3J0IEJhc2VWaWV3IGZyb20gJy4vYmFzZSc7XG5cbmV4cG9ydCBkZWZhdWx0IGNsYXNzIE1hdHJpeENvbHVtblNlbGVjdGlvblZpZXcgZXh0ZW5kcyBCYXNlVmlldyB7XG4gIGNvbnN0cnVjdG9yKG1hdHJpeCwgY29sdW1uSW5kaWNlcykge1xuICAgIGNoZWNrQ29sdW1uSW5kaWNlcyhtYXRyaXgsIGNvbHVtbkluZGljZXMpO1xuICAgIHN1cGVyKG1hdHJpeCwgbWF0cml4LnJvd3MsIGNvbHVtbkluZGljZXMubGVuZ3RoKTtcbiAgICB0aGlzLmNvbHVtbkluZGljZXMgPSBjb2x1bW5JbmRpY2VzO1xuICB9XG5cbiAgc2V0KHJvd0luZGV4LCBjb2x1bW5JbmRleCwgdmFsdWUpIHtcbiAgICB0aGlzLm1hdHJpeC5zZXQocm93SW5kZXgsIHRoaXMuY29sdW1uSW5kaWNlc1tjb2x1bW5JbmRleF0sIHZhbHVlKTtcbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIGdldChyb3dJbmRleCwgY29sdW1uSW5kZXgpIHtcbiAgICByZXR1cm4gdGhpcy5tYXRyaXguZ2V0KHJvd0luZGV4LCB0aGlzLmNvbHVtbkluZGljZXNbY29sdW1uSW5kZXhdKTtcbiAgfVxufVxuIiwiaW1wb3J0IEJhc2VWaWV3IGZyb20gJy4vYmFzZSc7XG5cbmV4cG9ydCBkZWZhdWx0IGNsYXNzIE1hdHJpeEZsaXBDb2x1bW5WaWV3IGV4dGVuZHMgQmFzZVZpZXcge1xuICBjb25zdHJ1Y3RvcihtYXRyaXgpIHtcbiAgICBzdXBlcihtYXRyaXgsIG1hdHJpeC5yb3dzLCBtYXRyaXguY29sdW1ucyk7XG4gIH1cblxuICBzZXQocm93SW5kZXgsIGNvbHVtbkluZGV4LCB2YWx1ZSkge1xuICAgIHRoaXMubWF0cml4LnNldChyb3dJbmRleCwgdGhpcy5jb2x1bW5zIC0gY29sdW1uSW5kZXggLSAxLCB2YWx1ZSk7XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBnZXQocm93SW5kZXgsIGNvbHVtbkluZGV4KSB7XG4gICAgcmV0dXJuIHRoaXMubWF0cml4LmdldChyb3dJbmRleCwgdGhpcy5jb2x1bW5zIC0gY29sdW1uSW5kZXggLSAxKTtcbiAgfVxufVxuIiwiaW1wb3J0IEJhc2VWaWV3IGZyb20gJy4vYmFzZSc7XG5cbmV4cG9ydCBkZWZhdWx0IGNsYXNzIE1hdHJpeEZsaXBSb3dWaWV3IGV4dGVuZHMgQmFzZVZpZXcge1xuICBjb25zdHJ1Y3RvcihtYXRyaXgpIHtcbiAgICBzdXBlcihtYXRyaXgsIG1hdHJpeC5yb3dzLCBtYXRyaXguY29sdW1ucyk7XG4gIH1cblxuICBzZXQocm93SW5kZXgsIGNvbHVtbkluZGV4LCB2YWx1ZSkge1xuICAgIHRoaXMubWF0cml4LnNldCh0aGlzLnJvd3MgLSByb3dJbmRleCAtIDEsIGNvbHVtbkluZGV4LCB2YWx1ZSk7XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBnZXQocm93SW5kZXgsIGNvbHVtbkluZGV4KSB7XG4gICAgcmV0dXJuIHRoaXMubWF0cml4LmdldCh0aGlzLnJvd3MgLSByb3dJbmRleCAtIDEsIGNvbHVtbkluZGV4KTtcbiAgfVxufVxuIiwiZXhwb3J0IHsgZGVmYXVsdCBhcyBNYXRyaXhDb2x1bW5WaWV3IH0gZnJvbSAnLi9jb2x1bW4nO1xuZXhwb3J0IHsgZGVmYXVsdCBhcyBNYXRyaXhDb2x1bW5TZWxlY3Rpb25WaWV3IH0gZnJvbSAnLi9jb2x1bW5TZWxlY3Rpb24nO1xuZXhwb3J0IHsgZGVmYXVsdCBhcyBNYXRyaXhGbGlwQ29sdW1uVmlldyB9IGZyb20gJy4vZmxpcENvbHVtbic7XG5leHBvcnQgeyBkZWZhdWx0IGFzIE1hdHJpeEZsaXBSb3dWaWV3IH0gZnJvbSAnLi9mbGlwUm93JztcbmV4cG9ydCB7IGRlZmF1bHQgYXMgTWF0cml4Um93VmlldyB9IGZyb20gJy4vcm93JztcbmV4cG9ydCB7IGRlZmF1bHQgYXMgTWF0cml4Um93U2VsZWN0aW9uVmlldyB9IGZyb20gJy4vcm93U2VsZWN0aW9uJztcbmV4cG9ydCB7IGRlZmF1bHQgYXMgTWF0cml4U2VsZWN0aW9uVmlldyB9IGZyb20gJy4vc2VsZWN0aW9uJztcbmV4cG9ydCB7IGRlZmF1bHQgYXMgTWF0cml4U3ViVmlldyB9IGZyb20gJy4vc3ViJztcbmV4cG9ydCB7IGRlZmF1bHQgYXMgTWF0cml4VHJhbnNwb3NlVmlldyB9IGZyb20gJy4vdHJhbnNwb3NlJztcbiIsImltcG9ydCB7IGNoZWNrUm93SW5kZXggfSBmcm9tICcuLi91dGlsJztcblxuaW1wb3J0IEJhc2VWaWV3IGZyb20gJy4vYmFzZSc7XG5cbmV4cG9ydCBkZWZhdWx0IGNsYXNzIE1hdHJpeFJvd1ZpZXcgZXh0ZW5kcyBCYXNlVmlldyB7XG4gIGNvbnN0cnVjdG9yKG1hdHJpeCwgcm93KSB7XG4gICAgY2hlY2tSb3dJbmRleChtYXRyaXgsIHJvdyk7XG4gICAgc3VwZXIobWF0cml4LCAxLCBtYXRyaXguY29sdW1ucyk7XG4gICAgdGhpcy5yb3cgPSByb3c7XG4gIH1cblxuICBzZXQocm93SW5kZXgsIGNvbHVtbkluZGV4LCB2YWx1ZSkge1xuICAgIHRoaXMubWF0cml4LnNldCh0aGlzLnJvdywgY29sdW1uSW5kZXgsIHZhbHVlKTtcbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIGdldChyb3dJbmRleCwgY29sdW1uSW5kZXgpIHtcbiAgICByZXR1cm4gdGhpcy5tYXRyaXguZ2V0KHRoaXMucm93LCBjb2x1bW5JbmRleCk7XG4gIH1cbn1cbiIsImltcG9ydCB7IGNoZWNrUm93SW5kaWNlcyB9IGZyb20gJy4uL3V0aWwnO1xuXG5pbXBvcnQgQmFzZVZpZXcgZnJvbSAnLi9iYXNlJztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgTWF0cml4Um93U2VsZWN0aW9uVmlldyBleHRlbmRzIEJhc2VWaWV3IHtcbiAgY29uc3RydWN0b3IobWF0cml4LCByb3dJbmRpY2VzKSB7XG4gICAgY2hlY2tSb3dJbmRpY2VzKG1hdHJpeCwgcm93SW5kaWNlcyk7XG4gICAgc3VwZXIobWF0cml4LCByb3dJbmRpY2VzLmxlbmd0aCwgbWF0cml4LmNvbHVtbnMpO1xuICAgIHRoaXMucm93SW5kaWNlcyA9IHJvd0luZGljZXM7XG4gIH1cblxuICBzZXQocm93SW5kZXgsIGNvbHVtbkluZGV4LCB2YWx1ZSkge1xuICAgIHRoaXMubWF0cml4LnNldCh0aGlzLnJvd0luZGljZXNbcm93SW5kZXhdLCBjb2x1bW5JbmRleCwgdmFsdWUpO1xuICAgIHJldHVybiB0aGlzO1xuICB9XG5cbiAgZ2V0KHJvd0luZGV4LCBjb2x1bW5JbmRleCkge1xuICAgIHJldHVybiB0aGlzLm1hdHJpeC5nZXQodGhpcy5yb3dJbmRpY2VzW3Jvd0luZGV4XSwgY29sdW1uSW5kZXgpO1xuICB9XG59XG4iLCJpbXBvcnQgeyBjaGVja1Jvd0luZGljZXMsIGNoZWNrQ29sdW1uSW5kaWNlcyB9IGZyb20gJy4uL3V0aWwnO1xuXG5pbXBvcnQgQmFzZVZpZXcgZnJvbSAnLi9iYXNlJztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgTWF0cml4U2VsZWN0aW9uVmlldyBleHRlbmRzIEJhc2VWaWV3IHtcbiAgY29uc3RydWN0b3IobWF0cml4LCByb3dJbmRpY2VzLCBjb2x1bW5JbmRpY2VzKSB7XG4gICAgY2hlY2tSb3dJbmRpY2VzKG1hdHJpeCwgcm93SW5kaWNlcyk7XG4gICAgY2hlY2tDb2x1bW5JbmRpY2VzKG1hdHJpeCwgY29sdW1uSW5kaWNlcyk7XG4gICAgc3VwZXIobWF0cml4LCByb3dJbmRpY2VzLmxlbmd0aCwgY29sdW1uSW5kaWNlcy5sZW5ndGgpO1xuICAgIHRoaXMucm93SW5kaWNlcyA9IHJvd0luZGljZXM7XG4gICAgdGhpcy5jb2x1bW5JbmRpY2VzID0gY29sdW1uSW5kaWNlcztcbiAgfVxuXG4gIHNldChyb3dJbmRleCwgY29sdW1uSW5kZXgsIHZhbHVlKSB7XG4gICAgdGhpcy5tYXRyaXguc2V0KFxuICAgICAgdGhpcy5yb3dJbmRpY2VzW3Jvd0luZGV4XSxcbiAgICAgIHRoaXMuY29sdW1uSW5kaWNlc1tjb2x1bW5JbmRleF0sXG4gICAgICB2YWx1ZSxcbiAgICApO1xuICAgIHJldHVybiB0aGlzO1xuICB9XG5cbiAgZ2V0KHJvd0luZGV4LCBjb2x1bW5JbmRleCkge1xuICAgIHJldHVybiB0aGlzLm1hdHJpeC5nZXQoXG4gICAgICB0aGlzLnJvd0luZGljZXNbcm93SW5kZXhdLFxuICAgICAgdGhpcy5jb2x1bW5JbmRpY2VzW2NvbHVtbkluZGV4XSxcbiAgICApO1xuICB9XG59XG4iLCJpbXBvcnQgeyBjaGVja1JhbmdlIH0gZnJvbSAnLi4vdXRpbCc7XG5cbmltcG9ydCBCYXNlVmlldyBmcm9tICcuL2Jhc2UnO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBNYXRyaXhTdWJWaWV3IGV4dGVuZHMgQmFzZVZpZXcge1xuICBjb25zdHJ1Y3RvcihtYXRyaXgsIHN0YXJ0Um93LCBlbmRSb3csIHN0YXJ0Q29sdW1uLCBlbmRDb2x1bW4pIHtcbiAgICBjaGVja1JhbmdlKG1hdHJpeCwgc3RhcnRSb3csIGVuZFJvdywgc3RhcnRDb2x1bW4sIGVuZENvbHVtbik7XG4gICAgc3VwZXIobWF0cml4LCBlbmRSb3cgLSBzdGFydFJvdyArIDEsIGVuZENvbHVtbiAtIHN0YXJ0Q29sdW1uICsgMSk7XG4gICAgdGhpcy5zdGFydFJvdyA9IHN0YXJ0Um93O1xuICAgIHRoaXMuc3RhcnRDb2x1bW4gPSBzdGFydENvbHVtbjtcbiAgfVxuXG4gIHNldChyb3dJbmRleCwgY29sdW1uSW5kZXgsIHZhbHVlKSB7XG4gICAgdGhpcy5tYXRyaXguc2V0KFxuICAgICAgdGhpcy5zdGFydFJvdyArIHJvd0luZGV4LFxuICAgICAgdGhpcy5zdGFydENvbHVtbiArIGNvbHVtbkluZGV4LFxuICAgICAgdmFsdWUsXG4gICAgKTtcbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIGdldChyb3dJbmRleCwgY29sdW1uSW5kZXgpIHtcbiAgICByZXR1cm4gdGhpcy5tYXRyaXguZ2V0KFxuICAgICAgdGhpcy5zdGFydFJvdyArIHJvd0luZGV4LFxuICAgICAgdGhpcy5zdGFydENvbHVtbiArIGNvbHVtbkluZGV4LFxuICAgICk7XG4gIH1cbn1cbiIsImltcG9ydCBCYXNlVmlldyBmcm9tICcuL2Jhc2UnO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBNYXRyaXhUcmFuc3Bvc2VWaWV3IGV4dGVuZHMgQmFzZVZpZXcge1xuICBjb25zdHJ1Y3RvcihtYXRyaXgpIHtcbiAgICBzdXBlcihtYXRyaXgsIG1hdHJpeC5jb2x1bW5zLCBtYXRyaXgucm93cyk7XG4gIH1cblxuICBzZXQocm93SW5kZXgsIGNvbHVtbkluZGV4LCB2YWx1ZSkge1xuICAgIHRoaXMubWF0cml4LnNldChjb2x1bW5JbmRleCwgcm93SW5kZXgsIHZhbHVlKTtcbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIGdldChyb3dJbmRleCwgY29sdW1uSW5kZXgpIHtcbiAgICByZXR1cm4gdGhpcy5tYXRyaXguZ2V0KGNvbHVtbkluZGV4LCByb3dJbmRleCk7XG4gIH1cbn1cbiIsImltcG9ydCB7IEFic3RyYWN0TWF0cml4IH0gZnJvbSAnLi4vbWF0cml4JztcblxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgV3JhcHBlck1hdHJpeDFEIGV4dGVuZHMgQWJzdHJhY3RNYXRyaXgge1xuICBjb25zdHJ1Y3RvcihkYXRhLCBvcHRpb25zID0ge30pIHtcbiAgICBjb25zdCB7IHJvd3MgPSAxIH0gPSBvcHRpb25zO1xuXG4gICAgaWYgKGRhdGEubGVuZ3RoICUgcm93cyAhPT0gMCkge1xuICAgICAgdGhyb3cgbmV3IEVycm9yKCd0aGUgZGF0YSBsZW5ndGggaXMgbm90IGRpdmlzaWJsZSBieSB0aGUgbnVtYmVyIG9mIHJvd3MnKTtcbiAgICB9XG4gICAgc3VwZXIoKTtcbiAgICB0aGlzLnJvd3MgPSByb3dzO1xuICAgIHRoaXMuY29sdW1ucyA9IGRhdGEubGVuZ3RoIC8gcm93cztcbiAgICB0aGlzLmRhdGEgPSBkYXRhO1xuICB9XG5cbiAgc2V0KHJvd0luZGV4LCBjb2x1bW5JbmRleCwgdmFsdWUpIHtcbiAgICBsZXQgaW5kZXggPSB0aGlzLl9jYWxjdWxhdGVJbmRleChyb3dJbmRleCwgY29sdW1uSW5kZXgpO1xuICAgIHRoaXMuZGF0YVtpbmRleF0gPSB2YWx1ZTtcbiAgICByZXR1cm4gdGhpcztcbiAgfVxuXG4gIGdldChyb3dJbmRleCwgY29sdW1uSW5kZXgpIHtcbiAgICBsZXQgaW5kZXggPSB0aGlzLl9jYWxjdWxhdGVJbmRleChyb3dJbmRleCwgY29sdW1uSW5kZXgpO1xuICAgIHJldHVybiB0aGlzLmRhdGFbaW5kZXhdO1xuICB9XG5cbiAgX2NhbGN1bGF0ZUluZGV4KHJvdywgY29sdW1uKSB7XG4gICAgcmV0dXJuIHJvdyAqIHRoaXMuY29sdW1ucyArIGNvbHVtbjtcbiAgfVxufVxuIiwiaW1wb3J0IHsgQWJzdHJhY3RNYXRyaXggfSBmcm9tICcuLi9tYXRyaXgnO1xuXG5leHBvcnQgZGVmYXVsdCBjbGFzcyBXcmFwcGVyTWF0cml4MkQgZXh0ZW5kcyBBYnN0cmFjdE1hdHJpeCB7XG4gIGNvbnN0cnVjdG9yKGRhdGEpIHtcbiAgICBzdXBlcigpO1xuICAgIHRoaXMuZGF0YSA9IGRhdGE7XG4gICAgdGhpcy5yb3dzID0gZGF0YS5sZW5ndGg7XG4gICAgdGhpcy5jb2x1bW5zID0gZGF0YVswXS5sZW5ndGg7XG4gIH1cblxuICBzZXQocm93SW5kZXgsIGNvbHVtbkluZGV4LCB2YWx1ZSkge1xuICAgIHRoaXMuZGF0YVtyb3dJbmRleF1bY29sdW1uSW5kZXhdID0gdmFsdWU7XG4gICAgcmV0dXJuIHRoaXM7XG4gIH1cblxuICBnZXQocm93SW5kZXgsIGNvbHVtbkluZGV4KSB7XG4gICAgcmV0dXJuIHRoaXMuZGF0YVtyb3dJbmRleF1bY29sdW1uSW5kZXhdO1xuICB9XG59XG4iLCJpbXBvcnQgeyBpc0FueUFycmF5IH0gZnJvbSAnaXMtYW55LWFycmF5JztcblxuaW1wb3J0IFdyYXBwZXJNYXRyaXgxRCBmcm9tICcuL1dyYXBwZXJNYXRyaXgxRCc7XG5pbXBvcnQgV3JhcHBlck1hdHJpeDJEIGZyb20gJy4vV3JhcHBlck1hdHJpeDJEJztcblxuZXhwb3J0IGZ1bmN0aW9uIHdyYXAoYXJyYXksIG9wdGlvbnMpIHtcbiAgaWYgKGlzQW55QXJyYXkoYXJyYXkpKSB7XG4gICAgaWYgKGFycmF5WzBdICYmIGlzQW55QXJyYXkoYXJyYXlbMF0pKSB7XG4gICAgICByZXR1cm4gbmV3IFdyYXBwZXJNYXRyaXgyRChhcnJheSk7XG4gICAgfSBlbHNlIHtcbiAgICAgIHJldHVybiBuZXcgV3JhcHBlck1hdHJpeDFEKGFycmF5LCBvcHRpb25zKTtcbiAgICB9XG4gIH0gZWxzZSB7XG4gICAgdGhyb3cgbmV3IEVycm9yKCd0aGUgYXJndW1lbnQgaXMgbm90IGFuIGFycmF5Jyk7XG4gIH1cbn1cbiIsIlxuZXhwb3J0IGRlZmF1bHQgY2xhc3MgVGlueVF1ZXVlIHtcbiAgICBjb25zdHJ1Y3RvcihkYXRhID0gW10sIGNvbXBhcmUgPSBkZWZhdWx0Q29tcGFyZSkge1xuICAgICAgICB0aGlzLmRhdGEgPSBkYXRhO1xuICAgICAgICB0aGlzLmxlbmd0aCA9IHRoaXMuZGF0YS5sZW5ndGg7XG4gICAgICAgIHRoaXMuY29tcGFyZSA9IGNvbXBhcmU7XG5cbiAgICAgICAgaWYgKHRoaXMubGVuZ3RoID4gMCkge1xuICAgICAgICAgICAgZm9yIChsZXQgaSA9ICh0aGlzLmxlbmd0aCA+PiAxKSAtIDE7IGkgPj0gMDsgaS0tKSB0aGlzLl9kb3duKGkpO1xuICAgICAgICB9XG4gICAgfVxuXG4gICAgcHVzaChpdGVtKSB7XG4gICAgICAgIHRoaXMuZGF0YS5wdXNoKGl0ZW0pO1xuICAgICAgICB0aGlzLmxlbmd0aCsrO1xuICAgICAgICB0aGlzLl91cCh0aGlzLmxlbmd0aCAtIDEpO1xuICAgIH1cblxuICAgIHBvcCgpIHtcbiAgICAgICAgaWYgKHRoaXMubGVuZ3RoID09PSAwKSByZXR1cm4gdW5kZWZpbmVkO1xuXG4gICAgICAgIGNvbnN0IHRvcCA9IHRoaXMuZGF0YVswXTtcbiAgICAgICAgY29uc3QgYm90dG9tID0gdGhpcy5kYXRhLnBvcCgpO1xuICAgICAgICB0aGlzLmxlbmd0aC0tO1xuXG4gICAgICAgIGlmICh0aGlzLmxlbmd0aCA+IDApIHtcbiAgICAgICAgICAgIHRoaXMuZGF0YVswXSA9IGJvdHRvbTtcbiAgICAgICAgICAgIHRoaXMuX2Rvd24oMCk7XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4gdG9wO1xuICAgIH1cblxuICAgIHBlZWsoKSB7XG4gICAgICAgIHJldHVybiB0aGlzLmRhdGFbMF07XG4gICAgfVxuXG4gICAgX3VwKHBvcykge1xuICAgICAgICBjb25zdCB7ZGF0YSwgY29tcGFyZX0gPSB0aGlzO1xuICAgICAgICBjb25zdCBpdGVtID0gZGF0YVtwb3NdO1xuXG4gICAgICAgIHdoaWxlIChwb3MgPiAwKSB7XG4gICAgICAgICAgICBjb25zdCBwYXJlbnQgPSAocG9zIC0gMSkgPj4gMTtcbiAgICAgICAgICAgIGNvbnN0IGN1cnJlbnQgPSBkYXRhW3BhcmVudF07XG4gICAgICAgICAgICBpZiAoY29tcGFyZShpdGVtLCBjdXJyZW50KSA+PSAwKSBicmVhaztcbiAgICAgICAgICAgIGRhdGFbcG9zXSA9IGN1cnJlbnQ7XG4gICAgICAgICAgICBwb3MgPSBwYXJlbnQ7XG4gICAgICAgIH1cblxuICAgICAgICBkYXRhW3Bvc10gPSBpdGVtO1xuICAgIH1cblxuICAgIF9kb3duKHBvcykge1xuICAgICAgICBjb25zdCB7ZGF0YSwgY29tcGFyZX0gPSB0aGlzO1xuICAgICAgICBjb25zdCBoYWxmTGVuZ3RoID0gdGhpcy5sZW5ndGggPj4gMTtcbiAgICAgICAgY29uc3QgaXRlbSA9IGRhdGFbcG9zXTtcblxuICAgICAgICB3aGlsZSAocG9zIDwgaGFsZkxlbmd0aCkge1xuICAgICAgICAgICAgbGV0IGxlZnQgPSAocG9zIDw8IDEpICsgMTtcbiAgICAgICAgICAgIGxldCBiZXN0ID0gZGF0YVtsZWZ0XTtcbiAgICAgICAgICAgIGNvbnN0IHJpZ2h0ID0gbGVmdCArIDE7XG5cbiAgICAgICAgICAgIGlmIChyaWdodCA8IHRoaXMubGVuZ3RoICYmIGNvbXBhcmUoZGF0YVtyaWdodF0sIGJlc3QpIDwgMCkge1xuICAgICAgICAgICAgICAgIGxlZnQgPSByaWdodDtcbiAgICAgICAgICAgICAgICBiZXN0ID0gZGF0YVtyaWdodF07XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpZiAoY29tcGFyZShiZXN0LCBpdGVtKSA+PSAwKSBicmVhaztcblxuICAgICAgICAgICAgZGF0YVtwb3NdID0gYmVzdDtcbiAgICAgICAgICAgIHBvcyA9IGxlZnQ7XG4gICAgICAgIH1cblxuICAgICAgICBkYXRhW3Bvc10gPSBpdGVtO1xuICAgIH1cbn1cblxuZnVuY3Rpb24gZGVmYXVsdENvbXBhcmUoYSwgYikge1xuICAgIHJldHVybiBhIDwgYiA/IC0xIDogYSA+IGIgPyAxIDogMDtcbn1cbiIsIi8vIFRoZSBtb2R1bGUgY2FjaGVcbnZhciBfX3dlYnBhY2tfbW9kdWxlX2NhY2hlX18gPSB7fTtcblxuLy8gVGhlIHJlcXVpcmUgZnVuY3Rpb25cbmZ1bmN0aW9uIF9fd2VicGFja19yZXF1aXJlX18obW9kdWxlSWQpIHtcblx0Ly8gQ2hlY2sgaWYgbW9kdWxlIGlzIGluIGNhY2hlXG5cdHZhciBjYWNoZWRNb2R1bGUgPSBfX3dlYnBhY2tfbW9kdWxlX2NhY2hlX19bbW9kdWxlSWRdO1xuXHRpZiAoY2FjaGVkTW9kdWxlICE9PSB1bmRlZmluZWQpIHtcblx0XHRyZXR1cm4gY2FjaGVkTW9kdWxlLmV4cG9ydHM7XG5cdH1cblx0Ly8gQ3JlYXRlIGEgbmV3IG1vZHVsZSAoYW5kIHB1dCBpdCBpbnRvIHRoZSBjYWNoZSlcblx0dmFyIG1vZHVsZSA9IF9fd2VicGFja19tb2R1bGVfY2FjaGVfX1ttb2R1bGVJZF0gPSB7XG5cdFx0Ly8gbm8gbW9kdWxlLmlkIG5lZWRlZFxuXHRcdC8vIG5vIG1vZHVsZS5sb2FkZWQgbmVlZGVkXG5cdFx0ZXhwb3J0czoge31cblx0fTtcblxuXHQvLyBFeGVjdXRlIHRoZSBtb2R1bGUgZnVuY3Rpb25cblx0X193ZWJwYWNrX21vZHVsZXNfX1ttb2R1bGVJZF0obW9kdWxlLCBtb2R1bGUuZXhwb3J0cywgX193ZWJwYWNrX3JlcXVpcmVfXyk7XG5cblx0Ly8gUmV0dXJuIHRoZSBleHBvcnRzIG9mIHRoZSBtb2R1bGVcblx0cmV0dXJuIG1vZHVsZS5leHBvcnRzO1xufVxuXG4iLCIvLyBkZWZpbmUgZ2V0dGVyIGZ1bmN0aW9ucyBmb3IgaGFybW9ueSBleHBvcnRzXG5fX3dlYnBhY2tfcmVxdWlyZV9fLmQgPSAoZXhwb3J0cywgZGVmaW5pdGlvbikgPT4ge1xuXHRmb3IodmFyIGtleSBpbiBkZWZpbml0aW9uKSB7XG5cdFx0aWYoX193ZWJwYWNrX3JlcXVpcmVfXy5vKGRlZmluaXRpb24sIGtleSkgJiYgIV9fd2VicGFja19yZXF1aXJlX18ubyhleHBvcnRzLCBrZXkpKSB7XG5cdFx0XHRPYmplY3QuZGVmaW5lUHJvcGVydHkoZXhwb3J0cywga2V5LCB7IGVudW1lcmFibGU6IHRydWUsIGdldDogZGVmaW5pdGlvbltrZXldIH0pO1xuXHRcdH1cblx0fVxufTsiLCJfX3dlYnBhY2tfcmVxdWlyZV9fLm8gPSAob2JqLCBwcm9wKSA9PiAoT2JqZWN0LnByb3RvdHlwZS5oYXNPd25Qcm9wZXJ0eS5jYWxsKG9iaiwgcHJvcCkpIiwiLy8gZGVmaW5lIF9fZXNNb2R1bGUgb24gZXhwb3J0c1xuX193ZWJwYWNrX3JlcXVpcmVfXy5yID0gKGV4cG9ydHMpID0+IHtcblx0aWYodHlwZW9mIFN5bWJvbCAhPT0gJ3VuZGVmaW5lZCcgJiYgU3ltYm9sLnRvU3RyaW5nVGFnKSB7XG5cdFx0T2JqZWN0LmRlZmluZVByb3BlcnR5KGV4cG9ydHMsIFN5bWJvbC50b1N0cmluZ1RhZywgeyB2YWx1ZTogJ01vZHVsZScgfSk7XG5cdH1cblx0T2JqZWN0LmRlZmluZVByb3BlcnR5KGV4cG9ydHMsICdfX2VzTW9kdWxlJywgeyB2YWx1ZTogdHJ1ZSB9KTtcbn07IiwiY29uc3Qge01hdGNoZXJ9ID0gcmVxdWlyZSgnLi9tYXRjaGluZy9tYXRjaGVyLmpzJyk7XG5jb25zdCB7RXN0aW1hdG9yfSA9IHJlcXVpcmUoJy4vZXN0aW1hdGlvbi9lc3RpbWF0b3IuanMnKTtcblxubGV0IHByb2plY3Rpb25UcmFuc2Zvcm0gPSBudWxsO1xubGV0IG1hdGNoaW5nRGF0YUxpc3QgPSBudWxsO1xubGV0IGRlYnVnTW9kZSA9IGZhbHNlO1xubGV0IG1hdGNoZXIgPSBudWxsO1xubGV0IGVzdGltYXRvciA9IG51bGw7XG5cbm9ubWVzc2FnZSA9IChtc2cpID0+IHtcbiAgY29uc3Qge2RhdGF9ID0gbXNnO1xuXG4gIGlmIChkYXRhLnR5cGUgPT09ICdzZXR1cCcpIHtcbiAgICBwcm9qZWN0aW9uVHJhbnNmb3JtID0gZGF0YS5wcm9qZWN0aW9uVHJhbnNmb3JtO1xuICAgIG1hdGNoaW5nRGF0YUxpc3QgPSBkYXRhLm1hdGNoaW5nRGF0YUxpc3Q7XG4gICAgZGVidWdNb2RlID0gZGF0YS5kZWJ1Z01vZGU7XG4gICAgbWF0Y2hlciA9IG5ldyBNYXRjaGVyKGRhdGEuaW5wdXRXaWR0aCwgZGF0YS5pbnB1dEhlaWdodCwgZGVidWdNb2RlKTtcbiAgICBlc3RpbWF0b3IgPSBuZXcgRXN0aW1hdG9yKGRhdGEucHJvamVjdGlvblRyYW5zZm9ybSk7XG4gIH1cbiAgZWxzZSBpZiAoZGF0YS50eXBlID09PSAnbWF0Y2gnKSB7XG4gICAgY29uc3QgaW50ZXJlc3RlZFRhcmdldEluZGV4ZXMgPSBkYXRhLnRhcmdldEluZGV4ZXM7XG5cbiAgICBsZXQgbWF0Y2hlZFRhcmdldEluZGV4ID0gLTE7XG4gICAgbGV0IG1hdGNoZWRNb2RlbFZpZXdUcmFuc2Zvcm0gPSBudWxsO1xuICAgIGxldCBtYXRjaGVkRGVidWdFeHRyYSA9IG51bGw7XG5cbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGludGVyZXN0ZWRUYXJnZXRJbmRleGVzLmxlbmd0aDsgaSsrKSB7XG4gICAgICBjb25zdCBtYXRjaGluZ0luZGV4ID0gaW50ZXJlc3RlZFRhcmdldEluZGV4ZXNbaV07XG5cbiAgICAgIGNvbnN0IHtrZXlmcmFtZUluZGV4LCBzY3JlZW5Db29yZHMsIHdvcmxkQ29vcmRzLCBkZWJ1Z0V4dHJhfSA9IG1hdGNoZXIubWF0Y2hEZXRlY3Rpb24obWF0Y2hpbmdEYXRhTGlzdFttYXRjaGluZ0luZGV4XSwgZGF0YS5mZWF0dXJlUG9pbnRzKTtcbiAgICAgIG1hdGNoZWREZWJ1Z0V4dHJhID0gZGVidWdFeHRyYTtcblxuICAgICAgaWYgKGtleWZyYW1lSW5kZXggIT09IC0xKSB7XG5cdGNvbnN0IG1vZGVsVmlld1RyYW5zZm9ybSA9IGVzdGltYXRvci5lc3RpbWF0ZSh7c2NyZWVuQ29vcmRzLCB3b3JsZENvb3Jkc30pO1xuXG5cdGlmIChtb2RlbFZpZXdUcmFuc2Zvcm0pIHtcblx0ICBtYXRjaGVkVGFyZ2V0SW5kZXggPSBtYXRjaGluZ0luZGV4O1xuXHQgIG1hdGNoZWRNb2RlbFZpZXdUcmFuc2Zvcm0gPSBtb2RlbFZpZXdUcmFuc2Zvcm07XG5cdH1cblx0YnJlYWs7XG4gICAgICB9XG4gICAgfVxuXG4gICAgcG9zdE1lc3NhZ2Uoe1xuICAgICAgdHlwZTogJ21hdGNoRG9uZScsXG4gICAgICB0YXJnZXRJbmRleDogbWF0Y2hlZFRhcmdldEluZGV4LFxuICAgICAgbW9kZWxWaWV3VHJhbnNmb3JtOiBtYXRjaGVkTW9kZWxWaWV3VHJhbnNmb3JtLFxuICAgICAgZGVidWdFeHRyYTogbWF0Y2hlZERlYnVnRXh0cmFcbiAgICB9KTtcbiAgfVxuICBlbHNlIGlmIChkYXRhLnR5cGUgPT09ICd0cmFja1VwZGF0ZScpIHtcbiAgICBjb25zdCB7bW9kZWxWaWV3VHJhbnNmb3JtLCB3b3JsZENvb3Jkcywgc2NyZWVuQ29vcmRzfSA9IGRhdGE7XG4gICAgY29uc3QgZmluYWxNb2RlbFZpZXdUcmFuc2Zvcm0gPSBlc3RpbWF0b3IucmVmaW5lRXN0aW1hdGUoe2luaXRpYWxNb2RlbFZpZXdUcmFuc2Zvcm06IG1vZGVsVmlld1RyYW5zZm9ybSwgd29ybGRDb29yZHMsIHNjcmVlbkNvb3Jkc30pO1xuICAgIHBvc3RNZXNzYWdlKHtcbiAgICAgIHR5cGU6ICd0cmFja1VwZGF0ZURvbmUnLFxuICAgICAgbW9kZWxWaWV3VHJhbnNmb3JtOiBmaW5hbE1vZGVsVmlld1RyYW5zZm9ybSxcbiAgICB9KTtcbiAgfVxufTtcblxuIl0sIm5hbWVzIjpbXSwic291cmNlUm9vdCI6IiJ9