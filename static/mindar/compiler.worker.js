/******/ (() => { // webpackBootstrap
/******/ 	var __webpack_modules__ = ({

/***/ "./mindar/image-target/image-list.js":
/*!*******************************************!*\
  !*** ./mindar/image-target/image-list.js ***!
  \*******************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {resize} = __webpack_require__(/*! ./utils/images.js */ "./mindar/image-target/utils/images.js");

const MIN_IMAGE_PIXEL_SIZE = 100;

// Build a list of image {data, width, height, scale} with different scales
const buildImageList = (inputImage) => {
  const minScale = MIN_IMAGE_PIXEL_SIZE / Math.min(inputImage.width, inputImage.height);

  const scaleList = [];
  let c = minScale;
  while (true) {
    scaleList.push(c);
    c *= Math.pow(2.0, 1.0/3.0);
    if (c >= 0.95) {
      c = 1;
      break;
    }
  }
  scaleList.push(c);
  scaleList.reverse();

  const imageList = [];
  for (let i = 0; i < scaleList.length; i++) {
    const w = inputImage.width * scaleList[i];
    const h = inputImage.height * scaleList[i];
    imageList.push(Object.assign(resize({image: inputImage, ratio: scaleList[i]}), {scale: scaleList[i]}));
  }
  return imageList;
}

const buildTrackingImageList = (inputImage) => {
  const minDimension = Math.min(inputImage.width, inputImage.height);
  const scaleList = [];
  const imageList = [];
  scaleList.push( 256.0 / minDimension);
  scaleList.push( 128.0 / minDimension);
  for (let i = 0; i < scaleList.length; i++) {
    imageList.push(Object.assign(resize({image: inputImage, ratio: scaleList[i]}), {scale: scaleList[i]}));
  }
  return imageList;
}

module.exports = {
  buildImageList,
  buildTrackingImageList
}


/***/ }),

/***/ "./mindar/image-target/tracker/extract.js":
/*!************************************************!*\
  !*** ./mindar/image-target/tracker/extract.js ***!
  \************************************************/
/***/ ((module, __unused_webpack_exports, __webpack_require__) => {

const {Cumsum} = __webpack_require__(/*! ../utils/cumsum */ "./mindar/image-target/utils/cumsum.js");

const SEARCH_SIZE1 = 10;
const SEARCH_SIZE2 = 2;

//const TEMPLATE_SIZE = 22 // DEFAULT
const TEMPLATE_SIZE = 6;
const TEMPLATE_SD_THRESH = 5.0;
const MAX_SIM_THRESH = 0.95;

const MAX_THRESH = 0.9;
//const MIN_THRESH = 0.55;
const MIN_THRESH = 0.2;
const SD_THRESH = 8.0;
const OCCUPANCY_SIZE = 24 * 2 / 3;

/*
 * Input image is in grey format. the imageData array size is width * height. value range from 0-255
 * pixel value at row r and c = imageData[r * width + c]
 *
 * @param {Uint8Array} options.imageData
 * @param {int} options.width image width
 * @param {int} options.height image height
 */
const extract = (image) => {
  const {data: imageData, width, height, scale} = image;

  // Step 1 - filter out interesting points. Interesting points have strong pixel value changed across neighbours
  const isPixelSelected = [width * height];
  for (let i = 0; i < isPixelSelected.length; i++) isPixelSelected[i] = false;

  // Step 1.1 consider a pixel at position (x, y). compute:
  //   dx = ((data[x+1, y-1] - data[x-1, y-1]) + (data[x+1, y] - data[x-1, y]) + (data[x+1, y+1] - data[x-1, y-1])) / 256 / 3
  //   dy = ((data[x+1, y+1] - data[x+1, y-1]) + (data[x, y+1] - data[x, y-1]) + (data[x-1, y+1] - data[x-1, y-1])) / 256 / 3
  //   dValue =  sqrt(dx^2 + dy^2) / 2;
  const dValue = new Float32Array(imageData.length);
  for (let i = 0; i < width; i++) {
    dValue[i] = -1;
    dValue[width * (height-1) + i] = -1;
  }
  for (let j = 0; j < height; j++) {
    dValue[j*width] = -1;
    dValue[j*width + width-1] = -1;
  }

  for (let i = 1; i < width-1; i++) {
    for (let j = 1; j < height-1; j++) {
      let pos = i + width * j;

      let dx = 0.0;
      let dy = 0.0;
      for (let k = -1; k <= 1; k++) {
        dx += (imageData[pos + width*k + 1] - imageData[pos + width*k -1]);
        dy += (imageData[pos + width + k] - imageData[pos - width + k]);
      }
      dx /= (3 * 256);
      dy /= (3 * 256);
      dValue[pos] = Math.sqrt( (dx * dx + dy * dy) / 2);
    }
  }

  // Step 1.2 - select all pixel which is dValue largest than all its neighbour as "potential" candidate
  //  the number of selected points is still too many, so we use the value to further filter (e.g. largest the dValue, the better)
  const dValueHist = new Uint32Array(1000); // histogram of dvalue scaled to [0, 1000)
  for (let i = 0; i < 1000; i++) dValueHist[i] = 0;
  const neighbourOffsets = [-1, 1, -width, width];
  let allCount = 0;
  for (let i = 1; i < width-1; i++) {
    for (let j = 1; j < height-1; j++) {
      let pos = i + width * j;
      let isMax = true;
      for (let d = 0; d < neighbourOffsets.length; d++) {
        if (dValue[pos] <= dValue[pos + neighbourOffsets[d]]) {
          isMax = false;
          break;
        }
      }
      if (isMax) {
        let k = Math.floor(dValue[pos] * 1000);
        if (k > 999) k = 999; // k>999 should not happen if computaiton is correction
        if (k < 0) k = 0; // k<0 should not happen if computaiton is correction
        dValueHist[k] += 1;
        allCount += 1;
        isPixelSelected[pos] = true;
      }
    }
  }

  // reduce number of points according to dValue.
  // actually, the whole Step 1. might be better to just sort the dvalues and pick the top (0.02 * width * height) points
  const maxPoints = 0.02 * width * height;
  let k = 999;
  let filteredCount = 0;
  while (k >= 0) {
    filteredCount += dValueHist[k];
    if (filteredCount > maxPoints) break;
    k--;
  }

  //console.log("image size: ", width * height);
  //console.log("extracted featues: ", allCount);
  //console.log("filtered featues: ", filteredCount);

  for (let i = 0; i < isPixelSelected.length; i++) {
    if (isPixelSelected[i]) {
      if (dValue[i] * 1000 < k) isPixelSelected[i] = false;
    }
  }

  //console.log("selected count: ", isPixelSelected.reduce((a, b) => {return a + (b?1:0);}, 0));

  // Step 2
  // prebuild cumulative sum matrix for fast computation
  const imageDataSqr = [];
  for (let i = 0; i < imageData.length; i++) {
    imageDataSqr[i] = imageData[i] * imageData[i];
  }
  const imageDataCumsum = new Cumsum(imageData, width, height);
  const imageDataSqrCumsum = new Cumsum(imageDataSqr, width, height);

  // holds the max similariliy value computed within SEARCH area of each pixel
  //   idea: if there is high simliarity with another pixel in nearby area, then it's not a good feature point
  //         next step is to find pixel with low similarity
  const featureMap = new Float32Array(imageData.length);

  for (let i = 0; i < width; i++) {
    for (let j = 0; j < height; j++) {
      const pos = j * width + i;
      if (!isPixelSelected[pos]) {
        featureMap[pos] = 1.0;
        continue;
      }

      const vlen = _templateVar({image, cx: i, cy: j, sdThresh: TEMPLATE_SD_THRESH, imageDataCumsum, imageDataSqrCumsum});
      if (vlen === null) {
        featureMap[pos] = 1.0;
        continue;
      }

      let max = -1.0;
      for (let jj = -SEARCH_SIZE1; jj <= SEARCH_SIZE1; jj++) {
        for (let ii = -SEARCH_SIZE1; ii <= SEARCH_SIZE1; ii++) {
          if (ii * ii + jj * jj <= SEARCH_SIZE2 * SEARCH_SIZE2) continue;
          const sim = _getSimilarity({image, cx: i+ii, cy: j+jj, vlen: vlen, tx: i, ty: j, imageDataCumsum, imageDataSqrCumsum});

          if (sim === null) continue;

          if (sim > max) {
            max = sim;
            if (max > MAX_SIM_THRESH) break;
          }
        }
        if (max > MAX_SIM_THRESH) break;
      }
      featureMap[pos] = max;
    }
  }

  // Step 2.2 select feature
  const coords = _selectFeature({image, featureMap, templateSize: TEMPLATE_SIZE, searchSize: SEARCH_SIZE2, occSize: OCCUPANCY_SIZE, maxSimThresh: MAX_THRESH, minSimThresh: MIN_THRESH, sdThresh: SD_THRESH, imageDataCumsum, imageDataSqrCumsum});

  return coords;
}

const _selectFeature = (options) => {
  let {image, featureMap, templateSize, searchSize, occSize, maxSimThresh, minSimThresh, sdThresh, imageDataCumsum, imageDataSqrCumsum} = options;
  const {data: imageData, width, height, scale} = image;

  //console.log("params: ", templateSize, templateSize, occSize, maxSimThresh, minSimThresh, sdThresh);

  //occSize *= 2;
  occSize = Math.floor(Math.min(image.width, image.height) / 10);

  const divSize = (templateSize * 2 + 1) * 3;
  const xDiv = Math.floor(width / divSize);
  const yDiv = Math.floor(height / divSize);

  let maxFeatureNum = Math.floor(width / occSize) * Math.floor(height / occSize) + xDiv * yDiv;
  //console.log("max feature num: ", maxFeatureNum);

  const coords = [];
  const image2 = new Float32Array(imageData.length);
  for (let i = 0; i < image2.length; i++) {
    image2[i] = featureMap[i];
  }

  let num = 0;
  while (num < maxFeatureNum) {
    let minSim = maxSimThresh;
    let cx = -1;
    let cy = -1;
    for (let j = 0; j < height; j++) {
      for (let i = 0; i < width; i++) {
        if (image2[j*width+i] < minSim) {
          minSim = image2[j*width+i];
          cx = i;
          cy = j;
        }
      }
    }
    if (cx === -1) break;

    const vlen = _templateVar({image, cx: cx, cy: cy, sdThresh: 0, imageDataCumsum, imageDataSqrCumsum});
    if (vlen === null) {
      image2[ cy * width + cx ] = 1.0;
      continue;
    }
    if (vlen / (templateSize * 2 + 1) < sdThresh) {
      image2[ cy * width + cx ] = 1.0;
      continue;
    }

    let min = 1.0;
    let max = -1.0;

    for (let j = -searchSize; j <= searchSize; j++) {
      for (let i = -searchSize; i <= searchSize; i++) {
        if (i*i + j*j > searchSize * searchSize) continue;
        if (i === 0 && j === 0) continue;

        const sim = _getSimilarity({image, vlen, cx: cx+i, cy: cy+j, tx: cx, ty:cy, imageDataCumsum, imageDataSqrCumsum});
        if (sim === null) continue;

        if (sim < min) {
          min = sim;
          if (min < minSimThresh && min < minSim) break;
        }
        if (sim > max) {
          max = sim;
          if (max > 0.99) break;
        }
      }
      if( (min < minSimThresh && min < minSim) || max > 0.99 ) break;
    }

    if( (min < minSimThresh && min < minSim) || max > 0.99 ) {
        image2[ cy * width + cx ] = 1.0;
        continue;
    }

    coords.push({x: cx, y: cy});
    //coords.push({
      //mx: 1.0 * cx / scale,
      //my: 1.0 * (height - cy) / scale,
    //})

    num += 1;
    //console.log(num, '(', cx, ',', cy, ')', minSim, 'min = ', min, 'max = ', max, 'sd = ', vlen/(templateSize*2+1));

    // no other feature points within occSize square
    for (let j = -occSize; j <= occSize; j++) {
      for (let i = -occSize; i <= occSize; i++) {
        if (cy + j < 0 || cy + j >= height || cx + i < 0 || cx + i >= width) continue;
        image2[ (cy+j)*width + (cx+i) ] = 1.0;
      }
    }
  }
  return coords;
}

// compute variances of the pixels, centered at (cx, cy)
const _templateVar = ({image, cx, cy, sdThresh, imageDataCumsum, imageDataSqrCumsum}) => {
  if (cx - TEMPLATE_SIZE < 0 || cx + TEMPLATE_SIZE >= image.width) return null;
  if (cy - TEMPLATE_SIZE < 0 || cy + TEMPLATE_SIZE >= image.height) return null;

  const templateWidth = 2 * TEMPLATE_SIZE + 1;
  const nPixels = templateWidth * templateWidth;

  let average = imageDataCumsum.query(cx - TEMPLATE_SIZE, cy - TEMPLATE_SIZE, cx + TEMPLATE_SIZE, cy+TEMPLATE_SIZE);
  average /= nPixels;

  //v = sum((pixel_i - avg)^2) for all pixel i within the template
  //  = sum(pixel_i^2) - sum(2 * avg * pixel_i) + sum(avg^avg)

  let vlen = imageDataSqrCumsum.query(cx - TEMPLATE_SIZE, cy - TEMPLATE_SIZE, cx + TEMPLATE_SIZE, cy+TEMPLATE_SIZE);
  vlen -= 2 * average * imageDataCumsum.query(cx - TEMPLATE_SIZE, cy - TEMPLATE_SIZE, cx + TEMPLATE_SIZE, cy+TEMPLATE_SIZE);
  vlen += nPixels * average * average;

  if (vlen / nPixels < sdThresh * sdThresh) return null;
  vlen = Math.sqrt(vlen);
  return vlen;
}

const _getSimilarity = (options) => {
  const {image, cx, cy, vlen, tx, ty, imageDataCumsum, imageDataSqrCumsum} = options;
  const {data: imageData, width, height} = image;
  const templateSize = TEMPLATE_SIZE;

  if (cx - templateSize < 0 || cx + templateSize >= width) return null;
  if (cy - templateSize < 0 || cy + templateSize >= height) return null;

  const templateWidth = 2 * templateSize + 1;

  let sx = imageDataCumsum.query(cx-templateSize, cy-templateSize, cx+templateSize, cy+templateSize);
  let sxx = imageDataSqrCumsum.query(cx-templateSize, cy-templateSize, cx+templateSize, cy+templateSize);
  let sxy = 0;

  // !! This loop is the performance bottleneck. Use moving pointers to optimize
  //
  //   for (let i = cx - templateSize, i2 = tx - templateSize; i <= cx + templateSize; i++, i2++) {
  //     for (let j = cy - templateSize, j2 = ty - templateSize; j <= cy + templateSize; j++, j2++) {
  //       sxy += imageData[j*width + i] * imageData[j2*width + i2];
  //     }
  //   }
  //
  let p1 = (cy-templateSize) * width + (cx-templateSize);
  let p2 = (ty-templateSize) * width + (tx-templateSize);
  let nextRowOffset = width - templateWidth;
  for (let j = 0; j < templateWidth; j++) {
    for (let i = 0; i < templateWidth; i++) {
      sxy += imageData[p1] * imageData[p2];
      p1 +=1;
      p2 +=1;
    }
    p1 += nextRowOffset;
    p2 += nextRowOffset;
  }

  let templateAverage = imageDataCumsum.query(tx-templateSize, ty-templateSize, tx+templateSize, ty+templateSize);
  templateAverage /= templateWidth * templateWidth;
  sxy -= templateAverage * sx;

  let vlen2 = sxx - sx*sx / (templateWidth * templateWidth);
  if (vlen2 == 0) return null;
  vlen2 = Math.sqrt(vlen2);

  // covariance between template and current pixel
  const sim = 1.0 * sxy / (vlen * vlen2);
  return sim;
}

module.exports = {
  extract
};


/***/ }),

/***/ "./mindar/image-target/utils/cumsum.js":
/*!*********************************************!*\
  !*** ./mindar/image-target/utils/cumsum.js ***!
  \*********************************************/
/***/ ((module) => {

// fast 2D submatrix sum using cumulative sum algorithm
class Cumsum {
  constructor(data, width, height) {
    this.cumsum = [];
    for (let j = 0; j < height; j++) {
      this.cumsum.push([]);
      for (let i = 0; i < width; i++) {
        this.cumsum[j].push(0);
      }
    }

    this.cumsum[0][0] = data[0];
    for (let i = 1; i < width; i++) {
      this.cumsum[0][i] = this.cumsum[0][i-1] + data[i];
    }
    for (let j = 1; j < height; j++) {
      this.cumsum[j][0] = this.cumsum[j-1][0] + data[j*width];
    }

    for (let j = 1; j < height; j++) {
      for (let i = 1; i < width; i++) {
        this.cumsum[j][i] = data[j*width+i]
                               + this.cumsum[j-1][i]
                               + this.cumsum[j][i-1]
                               - this.cumsum[j-1][i-1];
      }
    }
  }

  query(x1, y1, x2, y2) {
    let ret = this.cumsum[y2][x2];
    if (y1 > 0) ret -= this.cumsum[y1-1][x2];
    if (x1 > 0) ret -= this.cumsum[y2][x1-1];
    if (x1 > 0 && y1 > 0) ret += this.cumsum[y1-1][x1-1];
    return ret;
  }
}

module.exports = {
  Cumsum
}


/***/ }),

/***/ "./mindar/image-target/utils/images.js":
/*!*********************************************!*\
  !*** ./mindar/image-target/utils/images.js ***!
  \*********************************************/
/***/ ((module) => {

// simpler version of upsampling. better performance
const _upsampleBilinear = ({image, padOneWidth, padOneHeight}) => {
  const {width, height, data} = image;
  const dstWidth = image.width * 2 + (padOneWidth?1:0);
  const dstHeight = image.height * 2 + (padOneHeight?1:0);
  const temp = new Float32Array(dstWidth * dstHeight);

  for (let i = 0; i < width; i++) {
    for (let j = 0; j < height; j++) {
      const v = 0.25 * data[j * width + i];
      const ii = Math.floor(i/2);
      const jj = Math.floor(j/2);
      const pos = Math.floor(j/2) * dstWidth + Math.floor(i/2);
      temp[pos] += v;
      temp[pos+1] += v;
      temp[pos+dstWidth] += v;
      temp[pos+dstWidth+1] += v;
    }
  }
  return {data: temp, width: dstWidth, height: dstHeight};
}

// artoolkit version. slower. is it necessary?
const upsampleBilinear = ({image, padOneWidth, padOneHeight}) => {
  const {width, height, data} = image;

  const dstWidth = image.width * 2 + (padOneWidth?1:0);
  const dstHeight = image.height * 2 + (padOneHeight?1:0);

  const temp = new Float32Array(dstWidth * dstHeight);
  for (let i = 0; i < dstWidth; i++) {
    const si = 0.5 * i - 0.25;
    let si0 = Math.floor(si);
    let si1 = Math.ceil(si);
    if (si0 < 0) si0 = 0; // border
    if (si1 >= width) si1 = width - 1; // border

    for (let j = 0; j < dstHeight; j++) {
      const sj = 0.5 * j - 0.25;
      let sj0 = Math.floor(sj);
      let sj1 = Math.ceil(sj);
      if (sj0 < 0) sj0 = 0; // border
      if (sj1 >= height) sj1 = height - 1; //border

      const value = (si1 - si) * (sj1 - sj) * data[ sj0 * width + si0 ] +
                    (si1 - si) * (sj - sj0) * data[ sj1 * width + si0 ] +
                    (si - si0) * (sj1 - sj) * data[ sj0 * width + si1 ] +
                    (si - si0) * (sj - sj0) * data[ sj1 * width + si1 ];

      temp[j * dstWidth + i] = value;
    }
  }

  return {data: temp, width: dstWidth, height: dstHeight};
}

const downsampleBilinear = ({image}) => {
  const {data, width, height} = image;

  const dstWidth = Math.floor(width / 2);
  const dstHeight = Math.floor(height / 2);

  const temp = new Float32Array(dstWidth * dstHeight);
  const offsets = [0, 1, width, width+1];

  for (let j = 0; j < dstHeight; j++) {
    for (let i = 0; i < dstWidth; i++) {
      let srcPos = j*2 * width + i*2;
      let value = 0.0;
      for (let d = 0; d < offsets.length; d++) {
        value += data[srcPos+ offsets[d]];
      }
      value *= 0.25;
      temp[j*dstWidth+i] = value;
    }
  }
  return {data: temp, width: dstWidth, height: dstHeight};
}

const resize = ({image, ratio}) => {
  const width = Math.round(image.width * ratio);
  const height = Math.round(image.height * ratio);

  //const imageData = new Float32Array(width * height);
  const imageData = new Uint8Array(width * height);
  for (let i = 0; i < width; i++) {
    let si1 = Math.round(1.0 * i / ratio);
    let si2 = Math.round(1.0 * (i+1) / ratio) - 1;
    if (si2 >= image.width) si2 = image.width - 1;

    for (let j = 0; j < height; j++) {
      let sj1 = Math.round(1.0 * j / ratio);
      let sj2 = Math.round(1.0 * (j+1) / ratio) - 1;
      if (sj2 >= image.height) sj2 = image.height - 1;

      let sum = 0;
      let count = 0;
      for (let ii = si1; ii <= si2; ii++) {
        for (let jj = sj1; jj <= sj2; jj++) {
          sum += (1.0 * image.data[jj * image.width + ii]);
          count += 1;
        }
      }
      imageData[j * width + i] = Math.floor(sum / count);
    }
  }
  return {data: imageData, width: width, height: height};
}

module.exports = {
  downsampleBilinear,
  upsampleBilinear,
  resize,
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
var __webpack_exports__ = {};
// This entry need to be wrapped in an IIFE because it need to be isolated against other modules in the chunk.
(() => {
/*!************************************************!*\
  !*** ./mindar/image-target/compiler.worker.js ***!
  \************************************************/
const {extract} = __webpack_require__(/*! ./tracker/extract.js */ "./mindar/image-target/tracker/extract.js");
const {buildTrackingImageList} = __webpack_require__(/*! ./image-list.js */ "./mindar/image-target/image-list.js");

onmessage = (msg) => {
  const {data} = msg;
  if (data.type === 'compile') {
    //console.log("worker compile...");
    const {targetImages} = data;
    const percentPerImage = 50.0 / targetImages.length;
    let percent = 0.0;
    const list = [];
    for (let i = 0; i < targetImages.length; i++) {
      const targetImage = targetImages[i];
      const imageList = buildTrackingImageList(targetImage);
      const percentPerAction = percentPerImage / imageList.length;

      //console.log("compiling tracking...", i);
      const trackingData = _extractTrackingFeatures(imageList, (index) => {
	//console.log("done tracking", i, index);
	percent += percentPerAction
	postMessage({type: 'progress', percent});
      });
      list.push(trackingData);
    }
    postMessage({
      type: 'compileDone',
      list,
    });
  }
};

const _extractTrackingFeatures = (imageList, doneCallback) => {
  const featureSets = [];
  for (let i = 0; i < imageList.length; i++) {
    const image = imageList[i];
    const points = extract(image);

    const featureSet = {
      data: image.data,
      scale: image.scale,
      width: image.width,
      height: image.height,
      points,
    };
    featureSets.push(featureSet);

    doneCallback(i);
  }
  return featureSets;
}


})();

/******/ })()
;
//# sourceMappingURL=data:application/json;charset=utf-8;base64,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QUNBO0FBQ0E7Ozs7Ozs7O1VDakhBO1VBQ0E7O1VBRUE7VUFDQTtVQUNBO1VBQ0E7VUFDQTtVQUNBO1VBQ0E7VUFDQTtVQUNBO1VBQ0E7VUFDQTtVQUNBO1VBQ0E7O1VBRUE7VUFDQTs7VUFFQTtVQUNBO1VBQ0E7Ozs7Ozs7OztBQ3RCQSxPQUFPLFNBQVMsRUFBRSxtQkFBTyxDQUFDLHNFQUFzQjtBQUNoRCxPQUFPLHdCQUF3QixFQUFFLG1CQUFPLENBQUMsNERBQWlCOztBQUUxRDtBQUNBLFNBQVMsTUFBTTtBQUNmO0FBQ0E7QUFDQSxXQUFXLGNBQWM7QUFDekI7QUFDQTtBQUNBO0FBQ0Esb0JBQW9CLHlCQUF5QjtBQUM3QztBQUNBO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQSxjQUFjLDBCQUEwQjtBQUN4QyxPQUFPO0FBQ1A7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBLEtBQUs7QUFDTDtBQUNBOztBQUVBO0FBQ0E7QUFDQSxrQkFBa0Isc0JBQXNCO0FBQ3hDO0FBQ0E7O0FBRUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0E7QUFDQSIsInNvdXJjZXMiOlsid2VicGFjazovL3BhcmNlbC1hcHAvLi9taW5kYXIvaW1hZ2UtdGFyZ2V0L2ltYWdlLWxpc3QuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL21pbmRhci9pbWFnZS10YXJnZXQvdHJhY2tlci9leHRyYWN0LmpzIiwid2VicGFjazovL3BhcmNlbC1hcHAvLi9taW5kYXIvaW1hZ2UtdGFyZ2V0L3V0aWxzL2N1bXN1bS5qcyIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbWluZGFyL2ltYWdlLXRhcmdldC91dGlscy9pbWFnZXMuanMiLCJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC93ZWJwYWNrL2Jvb3RzdHJhcCIsIndlYnBhY2s6Ly9wYXJjZWwtYXBwLy4vbWluZGFyL2ltYWdlLXRhcmdldC9jb21waWxlci53b3JrZXIuanMiXSwic291cmNlc0NvbnRlbnQiOlsiY29uc3Qge3Jlc2l6ZX0gPSByZXF1aXJlKFwiLi91dGlscy9pbWFnZXMuanNcIik7XG5cbmNvbnN0IE1JTl9JTUFHRV9QSVhFTF9TSVpFID0gMTAwO1xuXG4vLyBCdWlsZCBhIGxpc3Qgb2YgaW1hZ2Uge2RhdGEsIHdpZHRoLCBoZWlnaHQsIHNjYWxlfSB3aXRoIGRpZmZlcmVudCBzY2FsZXNcbmNvbnN0IGJ1aWxkSW1hZ2VMaXN0ID0gKGlucHV0SW1hZ2UpID0+IHtcbiAgY29uc3QgbWluU2NhbGUgPSBNSU5fSU1BR0VfUElYRUxfU0laRSAvIE1hdGgubWluKGlucHV0SW1hZ2Uud2lkdGgsIGlucHV0SW1hZ2UuaGVpZ2h0KTtcblxuICBjb25zdCBzY2FsZUxpc3QgPSBbXTtcbiAgbGV0IGMgPSBtaW5TY2FsZTtcbiAgd2hpbGUgKHRydWUpIHtcbiAgICBzY2FsZUxpc3QucHVzaChjKTtcbiAgICBjICo9IE1hdGgucG93KDIuMCwgMS4wLzMuMCk7XG4gICAgaWYgKGMgPj0gMC45NSkge1xuICAgICAgYyA9IDE7XG4gICAgICBicmVhaztcbiAgICB9XG4gIH1cbiAgc2NhbGVMaXN0LnB1c2goYyk7XG4gIHNjYWxlTGlzdC5yZXZlcnNlKCk7XG5cbiAgY29uc3QgaW1hZ2VMaXN0ID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgc2NhbGVMaXN0Lmxlbmd0aDsgaSsrKSB7XG4gICAgY29uc3QgdyA9IGlucHV0SW1hZ2Uud2lkdGggKiBzY2FsZUxpc3RbaV07XG4gICAgY29uc3QgaCA9IGlucHV0SW1hZ2UuaGVpZ2h0ICogc2NhbGVMaXN0W2ldO1xuICAgIGltYWdlTGlzdC5wdXNoKE9iamVjdC5hc3NpZ24ocmVzaXplKHtpbWFnZTogaW5wdXRJbWFnZSwgcmF0aW86IHNjYWxlTGlzdFtpXX0pLCB7c2NhbGU6IHNjYWxlTGlzdFtpXX0pKTtcbiAgfVxuICByZXR1cm4gaW1hZ2VMaXN0O1xufVxuXG5jb25zdCBidWlsZFRyYWNraW5nSW1hZ2VMaXN0ID0gKGlucHV0SW1hZ2UpID0+IHtcbiAgY29uc3QgbWluRGltZW5zaW9uID0gTWF0aC5taW4oaW5wdXRJbWFnZS53aWR0aCwgaW5wdXRJbWFnZS5oZWlnaHQpO1xuICBjb25zdCBzY2FsZUxpc3QgPSBbXTtcbiAgY29uc3QgaW1hZ2VMaXN0ID0gW107XG4gIHNjYWxlTGlzdC5wdXNoKCAyNTYuMCAvIG1pbkRpbWVuc2lvbik7XG4gIHNjYWxlTGlzdC5wdXNoKCAxMjguMCAvIG1pbkRpbWVuc2lvbik7XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgc2NhbGVMaXN0Lmxlbmd0aDsgaSsrKSB7XG4gICAgaW1hZ2VMaXN0LnB1c2goT2JqZWN0LmFzc2lnbihyZXNpemUoe2ltYWdlOiBpbnB1dEltYWdlLCByYXRpbzogc2NhbGVMaXN0W2ldfSksIHtzY2FsZTogc2NhbGVMaXN0W2ldfSkpO1xuICB9XG4gIHJldHVybiBpbWFnZUxpc3Q7XG59XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuICBidWlsZEltYWdlTGlzdCxcbiAgYnVpbGRUcmFja2luZ0ltYWdlTGlzdFxufVxuIiwiY29uc3Qge0N1bXN1bX0gPSByZXF1aXJlKCcuLi91dGlscy9jdW1zdW0nKTtcblxuY29uc3QgU0VBUkNIX1NJWkUxID0gMTA7XG5jb25zdCBTRUFSQ0hfU0laRTIgPSAyO1xuXG4vL2NvbnN0IFRFTVBMQVRFX1NJWkUgPSAyMiAvLyBERUZBVUxUXG5jb25zdCBURU1QTEFURV9TSVpFID0gNjtcbmNvbnN0IFRFTVBMQVRFX1NEX1RIUkVTSCA9IDUuMDtcbmNvbnN0IE1BWF9TSU1fVEhSRVNIID0gMC45NTtcblxuY29uc3QgTUFYX1RIUkVTSCA9IDAuOTtcbi8vY29uc3QgTUlOX1RIUkVTSCA9IDAuNTU7XG5jb25zdCBNSU5fVEhSRVNIID0gMC4yO1xuY29uc3QgU0RfVEhSRVNIID0gOC4wO1xuY29uc3QgT0NDVVBBTkNZX1NJWkUgPSAyNCAqIDIgLyAzO1xuXG4vKlxuICogSW5wdXQgaW1hZ2UgaXMgaW4gZ3JleSBmb3JtYXQuIHRoZSBpbWFnZURhdGEgYXJyYXkgc2l6ZSBpcyB3aWR0aCAqIGhlaWdodC4gdmFsdWUgcmFuZ2UgZnJvbSAwLTI1NVxuICogcGl4ZWwgdmFsdWUgYXQgcm93IHIgYW5kIGMgPSBpbWFnZURhdGFbciAqIHdpZHRoICsgY11cbiAqXG4gKiBAcGFyYW0ge1VpbnQ4QXJyYXl9IG9wdGlvbnMuaW1hZ2VEYXRhXG4gKiBAcGFyYW0ge2ludH0gb3B0aW9ucy53aWR0aCBpbWFnZSB3aWR0aFxuICogQHBhcmFtIHtpbnR9IG9wdGlvbnMuaGVpZ2h0IGltYWdlIGhlaWdodFxuICovXG5jb25zdCBleHRyYWN0ID0gKGltYWdlKSA9PiB7XG4gIGNvbnN0IHtkYXRhOiBpbWFnZURhdGEsIHdpZHRoLCBoZWlnaHQsIHNjYWxlfSA9IGltYWdlO1xuXG4gIC8vIFN0ZXAgMSAtIGZpbHRlciBvdXQgaW50ZXJlc3RpbmcgcG9pbnRzLiBJbnRlcmVzdGluZyBwb2ludHMgaGF2ZSBzdHJvbmcgcGl4ZWwgdmFsdWUgY2hhbmdlZCBhY3Jvc3MgbmVpZ2hib3Vyc1xuICBjb25zdCBpc1BpeGVsU2VsZWN0ZWQgPSBbd2lkdGggKiBoZWlnaHRdO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGlzUGl4ZWxTZWxlY3RlZC5sZW5ndGg7IGkrKykgaXNQaXhlbFNlbGVjdGVkW2ldID0gZmFsc2U7XG5cbiAgLy8gU3RlcCAxLjEgY29uc2lkZXIgYSBwaXhlbCBhdCBwb3NpdGlvbiAoeCwgeSkuIGNvbXB1dGU6XG4gIC8vICAgZHggPSAoKGRhdGFbeCsxLCB5LTFdIC0gZGF0YVt4LTEsIHktMV0pICsgKGRhdGFbeCsxLCB5XSAtIGRhdGFbeC0xLCB5XSkgKyAoZGF0YVt4KzEsIHkrMV0gLSBkYXRhW3gtMSwgeS0xXSkpIC8gMjU2IC8gM1xuICAvLyAgIGR5ID0gKChkYXRhW3grMSwgeSsxXSAtIGRhdGFbeCsxLCB5LTFdKSArIChkYXRhW3gsIHkrMV0gLSBkYXRhW3gsIHktMV0pICsgKGRhdGFbeC0xLCB5KzFdIC0gZGF0YVt4LTEsIHktMV0pKSAvIDI1NiAvIDNcbiAgLy8gICBkVmFsdWUgPSAgc3FydChkeF4yICsgZHleMikgLyAyO1xuICBjb25zdCBkVmFsdWUgPSBuZXcgRmxvYXQzMkFycmF5KGltYWdlRGF0YS5sZW5ndGgpO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IHdpZHRoOyBpKyspIHtcbiAgICBkVmFsdWVbaV0gPSAtMTtcbiAgICBkVmFsdWVbd2lkdGggKiAoaGVpZ2h0LTEpICsgaV0gPSAtMTtcbiAgfVxuICBmb3IgKGxldCBqID0gMDsgaiA8IGhlaWdodDsgaisrKSB7XG4gICAgZFZhbHVlW2oqd2lkdGhdID0gLTE7XG4gICAgZFZhbHVlW2oqd2lkdGggKyB3aWR0aC0xXSA9IC0xO1xuICB9XG5cbiAgZm9yIChsZXQgaSA9IDE7IGkgPCB3aWR0aC0xOyBpKyspIHtcbiAgICBmb3IgKGxldCBqID0gMTsgaiA8IGhlaWdodC0xOyBqKyspIHtcbiAgICAgIGxldCBwb3MgPSBpICsgd2lkdGggKiBqO1xuXG4gICAgICBsZXQgZHggPSAwLjA7XG4gICAgICBsZXQgZHkgPSAwLjA7XG4gICAgICBmb3IgKGxldCBrID0gLTE7IGsgPD0gMTsgaysrKSB7XG4gICAgICAgIGR4ICs9IChpbWFnZURhdGFbcG9zICsgd2lkdGgqayArIDFdIC0gaW1hZ2VEYXRhW3BvcyArIHdpZHRoKmsgLTFdKTtcbiAgICAgICAgZHkgKz0gKGltYWdlRGF0YVtwb3MgKyB3aWR0aCArIGtdIC0gaW1hZ2VEYXRhW3BvcyAtIHdpZHRoICsga10pO1xuICAgICAgfVxuICAgICAgZHggLz0gKDMgKiAyNTYpO1xuICAgICAgZHkgLz0gKDMgKiAyNTYpO1xuICAgICAgZFZhbHVlW3Bvc10gPSBNYXRoLnNxcnQoIChkeCAqIGR4ICsgZHkgKiBkeSkgLyAyKTtcbiAgICB9XG4gIH1cblxuICAvLyBTdGVwIDEuMiAtIHNlbGVjdCBhbGwgcGl4ZWwgd2hpY2ggaXMgZFZhbHVlIGxhcmdlc3QgdGhhbiBhbGwgaXRzIG5laWdoYm91ciBhcyBcInBvdGVudGlhbFwiIGNhbmRpZGF0ZVxuICAvLyAgdGhlIG51bWJlciBvZiBzZWxlY3RlZCBwb2ludHMgaXMgc3RpbGwgdG9vIG1hbnksIHNvIHdlIHVzZSB0aGUgdmFsdWUgdG8gZnVydGhlciBmaWx0ZXIgKGUuZy4gbGFyZ2VzdCB0aGUgZFZhbHVlLCB0aGUgYmV0dGVyKVxuICBjb25zdCBkVmFsdWVIaXN0ID0gbmV3IFVpbnQzMkFycmF5KDEwMDApOyAvLyBoaXN0b2dyYW0gb2YgZHZhbHVlIHNjYWxlZCB0byBbMCwgMTAwMClcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCAxMDAwOyBpKyspIGRWYWx1ZUhpc3RbaV0gPSAwO1xuICBjb25zdCBuZWlnaGJvdXJPZmZzZXRzID0gWy0xLCAxLCAtd2lkdGgsIHdpZHRoXTtcbiAgbGV0IGFsbENvdW50ID0gMDtcbiAgZm9yIChsZXQgaSA9IDE7IGkgPCB3aWR0aC0xOyBpKyspIHtcbiAgICBmb3IgKGxldCBqID0gMTsgaiA8IGhlaWdodC0xOyBqKyspIHtcbiAgICAgIGxldCBwb3MgPSBpICsgd2lkdGggKiBqO1xuICAgICAgbGV0IGlzTWF4ID0gdHJ1ZTtcbiAgICAgIGZvciAobGV0IGQgPSAwOyBkIDwgbmVpZ2hib3VyT2Zmc2V0cy5sZW5ndGg7IGQrKykge1xuICAgICAgICBpZiAoZFZhbHVlW3Bvc10gPD0gZFZhbHVlW3BvcyArIG5laWdoYm91ck9mZnNldHNbZF1dKSB7XG4gICAgICAgICAgaXNNYXggPSBmYWxzZTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgfVxuICAgICAgaWYgKGlzTWF4KSB7XG4gICAgICAgIGxldCBrID0gTWF0aC5mbG9vcihkVmFsdWVbcG9zXSAqIDEwMDApO1xuICAgICAgICBpZiAoayA+IDk5OSkgayA9IDk5OTsgLy8gaz45OTkgc2hvdWxkIG5vdCBoYXBwZW4gaWYgY29tcHV0YWl0b24gaXMgY29ycmVjdGlvblxuICAgICAgICBpZiAoayA8IDApIGsgPSAwOyAvLyBrPDAgc2hvdWxkIG5vdCBoYXBwZW4gaWYgY29tcHV0YWl0b24gaXMgY29ycmVjdGlvblxuICAgICAgICBkVmFsdWVIaXN0W2tdICs9IDE7XG4gICAgICAgIGFsbENvdW50ICs9IDE7XG4gICAgICAgIGlzUGl4ZWxTZWxlY3RlZFtwb3NdID0gdHJ1ZTtcbiAgICAgIH1cbiAgICB9XG4gIH1cblxuICAvLyByZWR1Y2UgbnVtYmVyIG9mIHBvaW50cyBhY2NvcmRpbmcgdG8gZFZhbHVlLlxuICAvLyBhY3R1YWxseSwgdGhlIHdob2xlIFN0ZXAgMS4gbWlnaHQgYmUgYmV0dGVyIHRvIGp1c3Qgc29ydCB0aGUgZHZhbHVlcyBhbmQgcGljayB0aGUgdG9wICgwLjAyICogd2lkdGggKiBoZWlnaHQpIHBvaW50c1xuICBjb25zdCBtYXhQb2ludHMgPSAwLjAyICogd2lkdGggKiBoZWlnaHQ7XG4gIGxldCBrID0gOTk5O1xuICBsZXQgZmlsdGVyZWRDb3VudCA9IDA7XG4gIHdoaWxlIChrID49IDApIHtcbiAgICBmaWx0ZXJlZENvdW50ICs9IGRWYWx1ZUhpc3Rba107XG4gICAgaWYgKGZpbHRlcmVkQ291bnQgPiBtYXhQb2ludHMpIGJyZWFrO1xuICAgIGstLTtcbiAgfVxuXG4gIC8vY29uc29sZS5sb2coXCJpbWFnZSBzaXplOiBcIiwgd2lkdGggKiBoZWlnaHQpO1xuICAvL2NvbnNvbGUubG9nKFwiZXh0cmFjdGVkIGZlYXR1ZXM6IFwiLCBhbGxDb3VudCk7XG4gIC8vY29uc29sZS5sb2coXCJmaWx0ZXJlZCBmZWF0dWVzOiBcIiwgZmlsdGVyZWRDb3VudCk7XG5cbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBpc1BpeGVsU2VsZWN0ZWQubGVuZ3RoOyBpKyspIHtcbiAgICBpZiAoaXNQaXhlbFNlbGVjdGVkW2ldKSB7XG4gICAgICBpZiAoZFZhbHVlW2ldICogMTAwMCA8IGspIGlzUGl4ZWxTZWxlY3RlZFtpXSA9IGZhbHNlO1xuICAgIH1cbiAgfVxuXG4gIC8vY29uc29sZS5sb2coXCJzZWxlY3RlZCBjb3VudDogXCIsIGlzUGl4ZWxTZWxlY3RlZC5yZWR1Y2UoKGEsIGIpID0+IHtyZXR1cm4gYSArIChiPzE6MCk7fSwgMCkpO1xuXG4gIC8vIFN0ZXAgMlxuICAvLyBwcmVidWlsZCBjdW11bGF0aXZlIHN1bSBtYXRyaXggZm9yIGZhc3QgY29tcHV0YXRpb25cbiAgY29uc3QgaW1hZ2VEYXRhU3FyID0gW107XG4gIGZvciAobGV0IGkgPSAwOyBpIDwgaW1hZ2VEYXRhLmxlbmd0aDsgaSsrKSB7XG4gICAgaW1hZ2VEYXRhU3FyW2ldID0gaW1hZ2VEYXRhW2ldICogaW1hZ2VEYXRhW2ldO1xuICB9XG4gIGNvbnN0IGltYWdlRGF0YUN1bXN1bSA9IG5ldyBDdW1zdW0oaW1hZ2VEYXRhLCB3aWR0aCwgaGVpZ2h0KTtcbiAgY29uc3QgaW1hZ2VEYXRhU3FyQ3Vtc3VtID0gbmV3IEN1bXN1bShpbWFnZURhdGFTcXIsIHdpZHRoLCBoZWlnaHQpO1xuXG4gIC8vIGhvbGRzIHRoZSBtYXggc2ltaWxhcmlsaXkgdmFsdWUgY29tcHV0ZWQgd2l0aGluIFNFQVJDSCBhcmVhIG9mIGVhY2ggcGl4ZWxcbiAgLy8gICBpZGVhOiBpZiB0aGVyZSBpcyBoaWdoIHNpbWxpYXJpdHkgd2l0aCBhbm90aGVyIHBpeGVsIGluIG5lYXJieSBhcmVhLCB0aGVuIGl0J3Mgbm90IGEgZ29vZCBmZWF0dXJlIHBvaW50XG4gIC8vICAgICAgICAgbmV4dCBzdGVwIGlzIHRvIGZpbmQgcGl4ZWwgd2l0aCBsb3cgc2ltaWxhcml0eVxuICBjb25zdCBmZWF0dXJlTWFwID0gbmV3IEZsb2F0MzJBcnJheShpbWFnZURhdGEubGVuZ3RoKTtcblxuICBmb3IgKGxldCBpID0gMDsgaSA8IHdpZHRoOyBpKyspIHtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IGhlaWdodDsgaisrKSB7XG4gICAgICBjb25zdCBwb3MgPSBqICogd2lkdGggKyBpO1xuICAgICAgaWYgKCFpc1BpeGVsU2VsZWN0ZWRbcG9zXSkge1xuICAgICAgICBmZWF0dXJlTWFwW3Bvc10gPSAxLjA7XG4gICAgICAgIGNvbnRpbnVlO1xuICAgICAgfVxuXG4gICAgICBjb25zdCB2bGVuID0gX3RlbXBsYXRlVmFyKHtpbWFnZSwgY3g6IGksIGN5OiBqLCBzZFRocmVzaDogVEVNUExBVEVfU0RfVEhSRVNILCBpbWFnZURhdGFDdW1zdW0sIGltYWdlRGF0YVNxckN1bXN1bX0pO1xuICAgICAgaWYgKHZsZW4gPT09IG51bGwpIHtcbiAgICAgICAgZmVhdHVyZU1hcFtwb3NdID0gMS4wO1xuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgbGV0IG1heCA9IC0xLjA7XG4gICAgICBmb3IgKGxldCBqaiA9IC1TRUFSQ0hfU0laRTE7IGpqIDw9IFNFQVJDSF9TSVpFMTsgamorKykge1xuICAgICAgICBmb3IgKGxldCBpaSA9IC1TRUFSQ0hfU0laRTE7IGlpIDw9IFNFQVJDSF9TSVpFMTsgaWkrKykge1xuICAgICAgICAgIGlmIChpaSAqIGlpICsgamogKiBqaiA8PSBTRUFSQ0hfU0laRTIgKiBTRUFSQ0hfU0laRTIpIGNvbnRpbnVlO1xuICAgICAgICAgIGNvbnN0IHNpbSA9IF9nZXRTaW1pbGFyaXR5KHtpbWFnZSwgY3g6IGkraWksIGN5OiBqK2pqLCB2bGVuOiB2bGVuLCB0eDogaSwgdHk6IGosIGltYWdlRGF0YUN1bXN1bSwgaW1hZ2VEYXRhU3FyQ3Vtc3VtfSk7XG5cbiAgICAgICAgICBpZiAoc2ltID09PSBudWxsKSBjb250aW51ZTtcblxuICAgICAgICAgIGlmIChzaW0gPiBtYXgpIHtcbiAgICAgICAgICAgIG1heCA9IHNpbTtcbiAgICAgICAgICAgIGlmIChtYXggPiBNQVhfU0lNX1RIUkVTSCkgYnJlYWs7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIGlmIChtYXggPiBNQVhfU0lNX1RIUkVTSCkgYnJlYWs7XG4gICAgICB9XG4gICAgICBmZWF0dXJlTWFwW3Bvc10gPSBtYXg7XG4gICAgfVxuICB9XG5cbiAgLy8gU3RlcCAyLjIgc2VsZWN0IGZlYXR1cmVcbiAgY29uc3QgY29vcmRzID0gX3NlbGVjdEZlYXR1cmUoe2ltYWdlLCBmZWF0dXJlTWFwLCB0ZW1wbGF0ZVNpemU6IFRFTVBMQVRFX1NJWkUsIHNlYXJjaFNpemU6IFNFQVJDSF9TSVpFMiwgb2NjU2l6ZTogT0NDVVBBTkNZX1NJWkUsIG1heFNpbVRocmVzaDogTUFYX1RIUkVTSCwgbWluU2ltVGhyZXNoOiBNSU5fVEhSRVNILCBzZFRocmVzaDogU0RfVEhSRVNILCBpbWFnZURhdGFDdW1zdW0sIGltYWdlRGF0YVNxckN1bXN1bX0pO1xuXG4gIHJldHVybiBjb29yZHM7XG59XG5cbmNvbnN0IF9zZWxlY3RGZWF0dXJlID0gKG9wdGlvbnMpID0+IHtcbiAgbGV0IHtpbWFnZSwgZmVhdHVyZU1hcCwgdGVtcGxhdGVTaXplLCBzZWFyY2hTaXplLCBvY2NTaXplLCBtYXhTaW1UaHJlc2gsIG1pblNpbVRocmVzaCwgc2RUaHJlc2gsIGltYWdlRGF0YUN1bXN1bSwgaW1hZ2VEYXRhU3FyQ3Vtc3VtfSA9IG9wdGlvbnM7XG4gIGNvbnN0IHtkYXRhOiBpbWFnZURhdGEsIHdpZHRoLCBoZWlnaHQsIHNjYWxlfSA9IGltYWdlO1xuXG4gIC8vY29uc29sZS5sb2coXCJwYXJhbXM6IFwiLCB0ZW1wbGF0ZVNpemUsIHRlbXBsYXRlU2l6ZSwgb2NjU2l6ZSwgbWF4U2ltVGhyZXNoLCBtaW5TaW1UaHJlc2gsIHNkVGhyZXNoKTtcblxuICAvL29jY1NpemUgKj0gMjtcbiAgb2NjU2l6ZSA9IE1hdGguZmxvb3IoTWF0aC5taW4oaW1hZ2Uud2lkdGgsIGltYWdlLmhlaWdodCkgLyAxMCk7XG5cbiAgY29uc3QgZGl2U2l6ZSA9ICh0ZW1wbGF0ZVNpemUgKiAyICsgMSkgKiAzO1xuICBjb25zdCB4RGl2ID0gTWF0aC5mbG9vcih3aWR0aCAvIGRpdlNpemUpO1xuICBjb25zdCB5RGl2ID0gTWF0aC5mbG9vcihoZWlnaHQgLyBkaXZTaXplKTtcblxuICBsZXQgbWF4RmVhdHVyZU51bSA9IE1hdGguZmxvb3Iod2lkdGggLyBvY2NTaXplKSAqIE1hdGguZmxvb3IoaGVpZ2h0IC8gb2NjU2l6ZSkgKyB4RGl2ICogeURpdjtcbiAgLy9jb25zb2xlLmxvZyhcIm1heCBmZWF0dXJlIG51bTogXCIsIG1heEZlYXR1cmVOdW0pO1xuXG4gIGNvbnN0IGNvb3JkcyA9IFtdO1xuICBjb25zdCBpbWFnZTIgPSBuZXcgRmxvYXQzMkFycmF5KGltYWdlRGF0YS5sZW5ndGgpO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGltYWdlMi5sZW5ndGg7IGkrKykge1xuICAgIGltYWdlMltpXSA9IGZlYXR1cmVNYXBbaV07XG4gIH1cblxuICBsZXQgbnVtID0gMDtcbiAgd2hpbGUgKG51bSA8IG1heEZlYXR1cmVOdW0pIHtcbiAgICBsZXQgbWluU2ltID0gbWF4U2ltVGhyZXNoO1xuICAgIGxldCBjeCA9IC0xO1xuICAgIGxldCBjeSA9IC0xO1xuICAgIGZvciAobGV0IGogPSAwOyBqIDwgaGVpZ2h0OyBqKyspIHtcbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgd2lkdGg7IGkrKykge1xuICAgICAgICBpZiAoaW1hZ2UyW2oqd2lkdGgraV0gPCBtaW5TaW0pIHtcbiAgICAgICAgICBtaW5TaW0gPSBpbWFnZTJbaip3aWR0aCtpXTtcbiAgICAgICAgICBjeCA9IGk7XG4gICAgICAgICAgY3kgPSBqO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuICAgIGlmIChjeCA9PT0gLTEpIGJyZWFrO1xuXG4gICAgY29uc3QgdmxlbiA9IF90ZW1wbGF0ZVZhcih7aW1hZ2UsIGN4OiBjeCwgY3k6IGN5LCBzZFRocmVzaDogMCwgaW1hZ2VEYXRhQ3Vtc3VtLCBpbWFnZURhdGFTcXJDdW1zdW19KTtcbiAgICBpZiAodmxlbiA9PT0gbnVsbCkge1xuICAgICAgaW1hZ2UyWyBjeSAqIHdpZHRoICsgY3ggXSA9IDEuMDtcbiAgICAgIGNvbnRpbnVlO1xuICAgIH1cbiAgICBpZiAodmxlbiAvICh0ZW1wbGF0ZVNpemUgKiAyICsgMSkgPCBzZFRocmVzaCkge1xuICAgICAgaW1hZ2UyWyBjeSAqIHdpZHRoICsgY3ggXSA9IDEuMDtcbiAgICAgIGNvbnRpbnVlO1xuICAgIH1cblxuICAgIGxldCBtaW4gPSAxLjA7XG4gICAgbGV0IG1heCA9IC0xLjA7XG5cbiAgICBmb3IgKGxldCBqID0gLXNlYXJjaFNpemU7IGogPD0gc2VhcmNoU2l6ZTsgaisrKSB7XG4gICAgICBmb3IgKGxldCBpID0gLXNlYXJjaFNpemU7IGkgPD0gc2VhcmNoU2l6ZTsgaSsrKSB7XG4gICAgICAgIGlmIChpKmkgKyBqKmogPiBzZWFyY2hTaXplICogc2VhcmNoU2l6ZSkgY29udGludWU7XG4gICAgICAgIGlmIChpID09PSAwICYmIGogPT09IDApIGNvbnRpbnVlO1xuXG4gICAgICAgIGNvbnN0IHNpbSA9IF9nZXRTaW1pbGFyaXR5KHtpbWFnZSwgdmxlbiwgY3g6IGN4K2ksIGN5OiBjeStqLCB0eDogY3gsIHR5OmN5LCBpbWFnZURhdGFDdW1zdW0sIGltYWdlRGF0YVNxckN1bXN1bX0pO1xuICAgICAgICBpZiAoc2ltID09PSBudWxsKSBjb250aW51ZTtcblxuICAgICAgICBpZiAoc2ltIDwgbWluKSB7XG4gICAgICAgICAgbWluID0gc2ltO1xuICAgICAgICAgIGlmIChtaW4gPCBtaW5TaW1UaHJlc2ggJiYgbWluIDwgbWluU2ltKSBicmVhaztcbiAgICAgICAgfVxuICAgICAgICBpZiAoc2ltID4gbWF4KSB7XG4gICAgICAgICAgbWF4ID0gc2ltO1xuICAgICAgICAgIGlmIChtYXggPiAwLjk5KSBicmVhaztcbiAgICAgICAgfVxuICAgICAgfVxuICAgICAgaWYoIChtaW4gPCBtaW5TaW1UaHJlc2ggJiYgbWluIDwgbWluU2ltKSB8fCBtYXggPiAwLjk5ICkgYnJlYWs7XG4gICAgfVxuXG4gICAgaWYoIChtaW4gPCBtaW5TaW1UaHJlc2ggJiYgbWluIDwgbWluU2ltKSB8fCBtYXggPiAwLjk5ICkge1xuICAgICAgICBpbWFnZTJbIGN5ICogd2lkdGggKyBjeCBdID0gMS4wO1xuICAgICAgICBjb250aW51ZTtcbiAgICB9XG5cbiAgICBjb29yZHMucHVzaCh7eDogY3gsIHk6IGN5fSk7XG4gICAgLy9jb29yZHMucHVzaCh7XG4gICAgICAvL214OiAxLjAgKiBjeCAvIHNjYWxlLFxuICAgICAgLy9teTogMS4wICogKGhlaWdodCAtIGN5KSAvIHNjYWxlLFxuICAgIC8vfSlcblxuICAgIG51bSArPSAxO1xuICAgIC8vY29uc29sZS5sb2cobnVtLCAnKCcsIGN4LCAnLCcsIGN5LCAnKScsIG1pblNpbSwgJ21pbiA9ICcsIG1pbiwgJ21heCA9ICcsIG1heCwgJ3NkID0gJywgdmxlbi8odGVtcGxhdGVTaXplKjIrMSkpO1xuXG4gICAgLy8gbm8gb3RoZXIgZmVhdHVyZSBwb2ludHMgd2l0aGluIG9jY1NpemUgc3F1YXJlXG4gICAgZm9yIChsZXQgaiA9IC1vY2NTaXplOyBqIDw9IG9jY1NpemU7IGorKykge1xuICAgICAgZm9yIChsZXQgaSA9IC1vY2NTaXplOyBpIDw9IG9jY1NpemU7IGkrKykge1xuICAgICAgICBpZiAoY3kgKyBqIDwgMCB8fCBjeSArIGogPj0gaGVpZ2h0IHx8IGN4ICsgaSA8IDAgfHwgY3ggKyBpID49IHdpZHRoKSBjb250aW51ZTtcbiAgICAgICAgaW1hZ2UyWyAoY3kraikqd2lkdGggKyAoY3graSkgXSA9IDEuMDtcbiAgICAgIH1cbiAgICB9XG4gIH1cbiAgcmV0dXJuIGNvb3Jkcztcbn1cblxuLy8gY29tcHV0ZSB2YXJpYW5jZXMgb2YgdGhlIHBpeGVscywgY2VudGVyZWQgYXQgKGN4LCBjeSlcbmNvbnN0IF90ZW1wbGF0ZVZhciA9ICh7aW1hZ2UsIGN4LCBjeSwgc2RUaHJlc2gsIGltYWdlRGF0YUN1bXN1bSwgaW1hZ2VEYXRhU3FyQ3Vtc3VtfSkgPT4ge1xuICBpZiAoY3ggLSBURU1QTEFURV9TSVpFIDwgMCB8fCBjeCArIFRFTVBMQVRFX1NJWkUgPj0gaW1hZ2Uud2lkdGgpIHJldHVybiBudWxsO1xuICBpZiAoY3kgLSBURU1QTEFURV9TSVpFIDwgMCB8fCBjeSArIFRFTVBMQVRFX1NJWkUgPj0gaW1hZ2UuaGVpZ2h0KSByZXR1cm4gbnVsbDtcblxuICBjb25zdCB0ZW1wbGF0ZVdpZHRoID0gMiAqIFRFTVBMQVRFX1NJWkUgKyAxO1xuICBjb25zdCBuUGl4ZWxzID0gdGVtcGxhdGVXaWR0aCAqIHRlbXBsYXRlV2lkdGg7XG5cbiAgbGV0IGF2ZXJhZ2UgPSBpbWFnZURhdGFDdW1zdW0ucXVlcnkoY3ggLSBURU1QTEFURV9TSVpFLCBjeSAtIFRFTVBMQVRFX1NJWkUsIGN4ICsgVEVNUExBVEVfU0laRSwgY3krVEVNUExBVEVfU0laRSk7XG4gIGF2ZXJhZ2UgLz0gblBpeGVscztcblxuICAvL3YgPSBzdW0oKHBpeGVsX2kgLSBhdmcpXjIpIGZvciBhbGwgcGl4ZWwgaSB3aXRoaW4gdGhlIHRlbXBsYXRlXG4gIC8vICA9IHN1bShwaXhlbF9pXjIpIC0gc3VtKDIgKiBhdmcgKiBwaXhlbF9pKSArIHN1bShhdmdeYXZnKVxuXG4gIGxldCB2bGVuID0gaW1hZ2VEYXRhU3FyQ3Vtc3VtLnF1ZXJ5KGN4IC0gVEVNUExBVEVfU0laRSwgY3kgLSBURU1QTEFURV9TSVpFLCBjeCArIFRFTVBMQVRFX1NJWkUsIGN5K1RFTVBMQVRFX1NJWkUpO1xuICB2bGVuIC09IDIgKiBhdmVyYWdlICogaW1hZ2VEYXRhQ3Vtc3VtLnF1ZXJ5KGN4IC0gVEVNUExBVEVfU0laRSwgY3kgLSBURU1QTEFURV9TSVpFLCBjeCArIFRFTVBMQVRFX1NJWkUsIGN5K1RFTVBMQVRFX1NJWkUpO1xuICB2bGVuICs9IG5QaXhlbHMgKiBhdmVyYWdlICogYXZlcmFnZTtcblxuICBpZiAodmxlbiAvIG5QaXhlbHMgPCBzZFRocmVzaCAqIHNkVGhyZXNoKSByZXR1cm4gbnVsbDtcbiAgdmxlbiA9IE1hdGguc3FydCh2bGVuKTtcbiAgcmV0dXJuIHZsZW47XG59XG5cbmNvbnN0IF9nZXRTaW1pbGFyaXR5ID0gKG9wdGlvbnMpID0+IHtcbiAgY29uc3Qge2ltYWdlLCBjeCwgY3ksIHZsZW4sIHR4LCB0eSwgaW1hZ2VEYXRhQ3Vtc3VtLCBpbWFnZURhdGFTcXJDdW1zdW19ID0gb3B0aW9ucztcbiAgY29uc3Qge2RhdGE6IGltYWdlRGF0YSwgd2lkdGgsIGhlaWdodH0gPSBpbWFnZTtcbiAgY29uc3QgdGVtcGxhdGVTaXplID0gVEVNUExBVEVfU0laRTtcblxuICBpZiAoY3ggLSB0ZW1wbGF0ZVNpemUgPCAwIHx8IGN4ICsgdGVtcGxhdGVTaXplID49IHdpZHRoKSByZXR1cm4gbnVsbDtcbiAgaWYgKGN5IC0gdGVtcGxhdGVTaXplIDwgMCB8fCBjeSArIHRlbXBsYXRlU2l6ZSA+PSBoZWlnaHQpIHJldHVybiBudWxsO1xuXG4gIGNvbnN0IHRlbXBsYXRlV2lkdGggPSAyICogdGVtcGxhdGVTaXplICsgMTtcblxuICBsZXQgc3ggPSBpbWFnZURhdGFDdW1zdW0ucXVlcnkoY3gtdGVtcGxhdGVTaXplLCBjeS10ZW1wbGF0ZVNpemUsIGN4K3RlbXBsYXRlU2l6ZSwgY3krdGVtcGxhdGVTaXplKTtcbiAgbGV0IHN4eCA9IGltYWdlRGF0YVNxckN1bXN1bS5xdWVyeShjeC10ZW1wbGF0ZVNpemUsIGN5LXRlbXBsYXRlU2l6ZSwgY3grdGVtcGxhdGVTaXplLCBjeSt0ZW1wbGF0ZVNpemUpO1xuICBsZXQgc3h5ID0gMDtcblxuICAvLyAhISBUaGlzIGxvb3AgaXMgdGhlIHBlcmZvcm1hbmNlIGJvdHRsZW5lY2suIFVzZSBtb3ZpbmcgcG9pbnRlcnMgdG8gb3B0aW1pemVcbiAgLy9cbiAgLy8gICBmb3IgKGxldCBpID0gY3ggLSB0ZW1wbGF0ZVNpemUsIGkyID0gdHggLSB0ZW1wbGF0ZVNpemU7IGkgPD0gY3ggKyB0ZW1wbGF0ZVNpemU7IGkrKywgaTIrKykge1xuICAvLyAgICAgZm9yIChsZXQgaiA9IGN5IC0gdGVtcGxhdGVTaXplLCBqMiA9IHR5IC0gdGVtcGxhdGVTaXplOyBqIDw9IGN5ICsgdGVtcGxhdGVTaXplOyBqKyssIGoyKyspIHtcbiAgLy8gICAgICAgc3h5ICs9IGltYWdlRGF0YVtqKndpZHRoICsgaV0gKiBpbWFnZURhdGFbajIqd2lkdGggKyBpMl07XG4gIC8vICAgICB9XG4gIC8vICAgfVxuICAvL1xuICBsZXQgcDEgPSAoY3ktdGVtcGxhdGVTaXplKSAqIHdpZHRoICsgKGN4LXRlbXBsYXRlU2l6ZSk7XG4gIGxldCBwMiA9ICh0eS10ZW1wbGF0ZVNpemUpICogd2lkdGggKyAodHgtdGVtcGxhdGVTaXplKTtcbiAgbGV0IG5leHRSb3dPZmZzZXQgPSB3aWR0aCAtIHRlbXBsYXRlV2lkdGg7XG4gIGZvciAobGV0IGogPSAwOyBqIDwgdGVtcGxhdGVXaWR0aDsgaisrKSB7XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0ZW1wbGF0ZVdpZHRoOyBpKyspIHtcbiAgICAgIHN4eSArPSBpbWFnZURhdGFbcDFdICogaW1hZ2VEYXRhW3AyXTtcbiAgICAgIHAxICs9MTtcbiAgICAgIHAyICs9MTtcbiAgICB9XG4gICAgcDEgKz0gbmV4dFJvd09mZnNldDtcbiAgICBwMiArPSBuZXh0Um93T2Zmc2V0O1xuICB9XG5cbiAgbGV0IHRlbXBsYXRlQXZlcmFnZSA9IGltYWdlRGF0YUN1bXN1bS5xdWVyeSh0eC10ZW1wbGF0ZVNpemUsIHR5LXRlbXBsYXRlU2l6ZSwgdHgrdGVtcGxhdGVTaXplLCB0eSt0ZW1wbGF0ZVNpemUpO1xuICB0ZW1wbGF0ZUF2ZXJhZ2UgLz0gdGVtcGxhdGVXaWR0aCAqIHRlbXBsYXRlV2lkdGg7XG4gIHN4eSAtPSB0ZW1wbGF0ZUF2ZXJhZ2UgKiBzeDtcblxuICBsZXQgdmxlbjIgPSBzeHggLSBzeCpzeCAvICh0ZW1wbGF0ZVdpZHRoICogdGVtcGxhdGVXaWR0aCk7XG4gIGlmICh2bGVuMiA9PSAwKSByZXR1cm4gbnVsbDtcbiAgdmxlbjIgPSBNYXRoLnNxcnQodmxlbjIpO1xuXG4gIC8vIGNvdmFyaWFuY2UgYmV0d2VlbiB0ZW1wbGF0ZSBhbmQgY3VycmVudCBwaXhlbFxuICBjb25zdCBzaW0gPSAxLjAgKiBzeHkgLyAodmxlbiAqIHZsZW4yKTtcbiAgcmV0dXJuIHNpbTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSB7XG4gIGV4dHJhY3Rcbn07XG4iLCIvLyBmYXN0IDJEIHN1Ym1hdHJpeCBzdW0gdXNpbmcgY3VtdWxhdGl2ZSBzdW0gYWxnb3JpdGhtXG5jbGFzcyBDdW1zdW0ge1xuICBjb25zdHJ1Y3RvcihkYXRhLCB3aWR0aCwgaGVpZ2h0KSB7XG4gICAgdGhpcy5jdW1zdW0gPSBbXTtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IGhlaWdodDsgaisrKSB7XG4gICAgICB0aGlzLmN1bXN1bS5wdXNoKFtdKTtcbiAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgd2lkdGg7IGkrKykge1xuICAgICAgICB0aGlzLmN1bXN1bVtqXS5wdXNoKDApO1xuICAgICAgfVxuICAgIH1cblxuICAgIHRoaXMuY3Vtc3VtWzBdWzBdID0gZGF0YVswXTtcbiAgICBmb3IgKGxldCBpID0gMTsgaSA8IHdpZHRoOyBpKyspIHtcbiAgICAgIHRoaXMuY3Vtc3VtWzBdW2ldID0gdGhpcy5jdW1zdW1bMF1baS0xXSArIGRhdGFbaV07XG4gICAgfVxuICAgIGZvciAobGV0IGogPSAxOyBqIDwgaGVpZ2h0OyBqKyspIHtcbiAgICAgIHRoaXMuY3Vtc3VtW2pdWzBdID0gdGhpcy5jdW1zdW1bai0xXVswXSArIGRhdGFbaip3aWR0aF07XG4gICAgfVxuXG4gICAgZm9yIChsZXQgaiA9IDE7IGogPCBoZWlnaHQ7IGorKykge1xuICAgICAgZm9yIChsZXQgaSA9IDE7IGkgPCB3aWR0aDsgaSsrKSB7XG4gICAgICAgIHRoaXMuY3Vtc3VtW2pdW2ldID0gZGF0YVtqKndpZHRoK2ldXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgKyB0aGlzLmN1bXN1bVtqLTFdW2ldXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgKyB0aGlzLmN1bXN1bVtqXVtpLTFdXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgLSB0aGlzLmN1bXN1bVtqLTFdW2ktMV07XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgcXVlcnkoeDEsIHkxLCB4MiwgeTIpIHtcbiAgICBsZXQgcmV0ID0gdGhpcy5jdW1zdW1beTJdW3gyXTtcbiAgICBpZiAoeTEgPiAwKSByZXQgLT0gdGhpcy5jdW1zdW1beTEtMV1beDJdO1xuICAgIGlmICh4MSA+IDApIHJldCAtPSB0aGlzLmN1bXN1bVt5Ml1beDEtMV07XG4gICAgaWYgKHgxID4gMCAmJiB5MSA+IDApIHJldCArPSB0aGlzLmN1bXN1bVt5MS0xXVt4MS0xXTtcbiAgICByZXR1cm4gcmV0O1xuICB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0ge1xuICBDdW1zdW1cbn1cbiIsIi8vIHNpbXBsZXIgdmVyc2lvbiBvZiB1cHNhbXBsaW5nLiBiZXR0ZXIgcGVyZm9ybWFuY2VcbmNvbnN0IF91cHNhbXBsZUJpbGluZWFyID0gKHtpbWFnZSwgcGFkT25lV2lkdGgsIHBhZE9uZUhlaWdodH0pID0+IHtcbiAgY29uc3Qge3dpZHRoLCBoZWlnaHQsIGRhdGF9ID0gaW1hZ2U7XG4gIGNvbnN0IGRzdFdpZHRoID0gaW1hZ2Uud2lkdGggKiAyICsgKHBhZE9uZVdpZHRoPzE6MCk7XG4gIGNvbnN0IGRzdEhlaWdodCA9IGltYWdlLmhlaWdodCAqIDIgKyAocGFkT25lSGVpZ2h0PzE6MCk7XG4gIGNvbnN0IHRlbXAgPSBuZXcgRmxvYXQzMkFycmF5KGRzdFdpZHRoICogZHN0SGVpZ2h0KTtcblxuICBmb3IgKGxldCBpID0gMDsgaSA8IHdpZHRoOyBpKyspIHtcbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IGhlaWdodDsgaisrKSB7XG4gICAgICBjb25zdCB2ID0gMC4yNSAqIGRhdGFbaiAqIHdpZHRoICsgaV07XG4gICAgICBjb25zdCBpaSA9IE1hdGguZmxvb3IoaS8yKTtcbiAgICAgIGNvbnN0IGpqID0gTWF0aC5mbG9vcihqLzIpO1xuICAgICAgY29uc3QgcG9zID0gTWF0aC5mbG9vcihqLzIpICogZHN0V2lkdGggKyBNYXRoLmZsb29yKGkvMik7XG4gICAgICB0ZW1wW3Bvc10gKz0gdjtcbiAgICAgIHRlbXBbcG9zKzFdICs9IHY7XG4gICAgICB0ZW1wW3Bvcytkc3RXaWR0aF0gKz0gdjtcbiAgICAgIHRlbXBbcG9zK2RzdFdpZHRoKzFdICs9IHY7XG4gICAgfVxuICB9XG4gIHJldHVybiB7ZGF0YTogdGVtcCwgd2lkdGg6IGRzdFdpZHRoLCBoZWlnaHQ6IGRzdEhlaWdodH07XG59XG5cbi8vIGFydG9vbGtpdCB2ZXJzaW9uLiBzbG93ZXIuIGlzIGl0IG5lY2Vzc2FyeT9cbmNvbnN0IHVwc2FtcGxlQmlsaW5lYXIgPSAoe2ltYWdlLCBwYWRPbmVXaWR0aCwgcGFkT25lSGVpZ2h0fSkgPT4ge1xuICBjb25zdCB7d2lkdGgsIGhlaWdodCwgZGF0YX0gPSBpbWFnZTtcblxuICBjb25zdCBkc3RXaWR0aCA9IGltYWdlLndpZHRoICogMiArIChwYWRPbmVXaWR0aD8xOjApO1xuICBjb25zdCBkc3RIZWlnaHQgPSBpbWFnZS5oZWlnaHQgKiAyICsgKHBhZE9uZUhlaWdodD8xOjApO1xuXG4gIGNvbnN0IHRlbXAgPSBuZXcgRmxvYXQzMkFycmF5KGRzdFdpZHRoICogZHN0SGVpZ2h0KTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCBkc3RXaWR0aDsgaSsrKSB7XG4gICAgY29uc3Qgc2kgPSAwLjUgKiBpIC0gMC4yNTtcbiAgICBsZXQgc2kwID0gTWF0aC5mbG9vcihzaSk7XG4gICAgbGV0IHNpMSA9IE1hdGguY2VpbChzaSk7XG4gICAgaWYgKHNpMCA8IDApIHNpMCA9IDA7IC8vIGJvcmRlclxuICAgIGlmIChzaTEgPj0gd2lkdGgpIHNpMSA9IHdpZHRoIC0gMTsgLy8gYm9yZGVyXG5cbiAgICBmb3IgKGxldCBqID0gMDsgaiA8IGRzdEhlaWdodDsgaisrKSB7XG4gICAgICBjb25zdCBzaiA9IDAuNSAqIGogLSAwLjI1O1xuICAgICAgbGV0IHNqMCA9IE1hdGguZmxvb3Ioc2opO1xuICAgICAgbGV0IHNqMSA9IE1hdGguY2VpbChzaik7XG4gICAgICBpZiAoc2owIDwgMCkgc2owID0gMDsgLy8gYm9yZGVyXG4gICAgICBpZiAoc2oxID49IGhlaWdodCkgc2oxID0gaGVpZ2h0IC0gMTsgLy9ib3JkZXJcblxuICAgICAgY29uc3QgdmFsdWUgPSAoc2kxIC0gc2kpICogKHNqMSAtIHNqKSAqIGRhdGFbIHNqMCAqIHdpZHRoICsgc2kwIF0gK1xuICAgICAgICAgICAgICAgICAgICAoc2kxIC0gc2kpICogKHNqIC0gc2owKSAqIGRhdGFbIHNqMSAqIHdpZHRoICsgc2kwIF0gK1xuICAgICAgICAgICAgICAgICAgICAoc2kgLSBzaTApICogKHNqMSAtIHNqKSAqIGRhdGFbIHNqMCAqIHdpZHRoICsgc2kxIF0gK1xuICAgICAgICAgICAgICAgICAgICAoc2kgLSBzaTApICogKHNqIC0gc2owKSAqIGRhdGFbIHNqMSAqIHdpZHRoICsgc2kxIF07XG5cbiAgICAgIHRlbXBbaiAqIGRzdFdpZHRoICsgaV0gPSB2YWx1ZTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4ge2RhdGE6IHRlbXAsIHdpZHRoOiBkc3RXaWR0aCwgaGVpZ2h0OiBkc3RIZWlnaHR9O1xufVxuXG5jb25zdCBkb3duc2FtcGxlQmlsaW5lYXIgPSAoe2ltYWdlfSkgPT4ge1xuICBjb25zdCB7ZGF0YSwgd2lkdGgsIGhlaWdodH0gPSBpbWFnZTtcblxuICBjb25zdCBkc3RXaWR0aCA9IE1hdGguZmxvb3Iod2lkdGggLyAyKTtcbiAgY29uc3QgZHN0SGVpZ2h0ID0gTWF0aC5mbG9vcihoZWlnaHQgLyAyKTtcblxuICBjb25zdCB0ZW1wID0gbmV3IEZsb2F0MzJBcnJheShkc3RXaWR0aCAqIGRzdEhlaWdodCk7XG4gIGNvbnN0IG9mZnNldHMgPSBbMCwgMSwgd2lkdGgsIHdpZHRoKzFdO1xuXG4gIGZvciAobGV0IGogPSAwOyBqIDwgZHN0SGVpZ2h0OyBqKyspIHtcbiAgICBmb3IgKGxldCBpID0gMDsgaSA8IGRzdFdpZHRoOyBpKyspIHtcbiAgICAgIGxldCBzcmNQb3MgPSBqKjIgKiB3aWR0aCArIGkqMjtcbiAgICAgIGxldCB2YWx1ZSA9IDAuMDtcbiAgICAgIGZvciAobGV0IGQgPSAwOyBkIDwgb2Zmc2V0cy5sZW5ndGg7IGQrKykge1xuICAgICAgICB2YWx1ZSArPSBkYXRhW3NyY1Bvcysgb2Zmc2V0c1tkXV07XG4gICAgICB9XG4gICAgICB2YWx1ZSAqPSAwLjI1O1xuICAgICAgdGVtcFtqKmRzdFdpZHRoK2ldID0gdmFsdWU7XG4gICAgfVxuICB9XG4gIHJldHVybiB7ZGF0YTogdGVtcCwgd2lkdGg6IGRzdFdpZHRoLCBoZWlnaHQ6IGRzdEhlaWdodH07XG59XG5cbmNvbnN0IHJlc2l6ZSA9ICh7aW1hZ2UsIHJhdGlvfSkgPT4ge1xuICBjb25zdCB3aWR0aCA9IE1hdGgucm91bmQoaW1hZ2Uud2lkdGggKiByYXRpbyk7XG4gIGNvbnN0IGhlaWdodCA9IE1hdGgucm91bmQoaW1hZ2UuaGVpZ2h0ICogcmF0aW8pO1xuXG4gIC8vY29uc3QgaW1hZ2VEYXRhID0gbmV3IEZsb2F0MzJBcnJheSh3aWR0aCAqIGhlaWdodCk7XG4gIGNvbnN0IGltYWdlRGF0YSA9IG5ldyBVaW50OEFycmF5KHdpZHRoICogaGVpZ2h0KTtcbiAgZm9yIChsZXQgaSA9IDA7IGkgPCB3aWR0aDsgaSsrKSB7XG4gICAgbGV0IHNpMSA9IE1hdGgucm91bmQoMS4wICogaSAvIHJhdGlvKTtcbiAgICBsZXQgc2kyID0gTWF0aC5yb3VuZCgxLjAgKiAoaSsxKSAvIHJhdGlvKSAtIDE7XG4gICAgaWYgKHNpMiA+PSBpbWFnZS53aWR0aCkgc2kyID0gaW1hZ2Uud2lkdGggLSAxO1xuXG4gICAgZm9yIChsZXQgaiA9IDA7IGogPCBoZWlnaHQ7IGorKykge1xuICAgICAgbGV0IHNqMSA9IE1hdGgucm91bmQoMS4wICogaiAvIHJhdGlvKTtcbiAgICAgIGxldCBzajIgPSBNYXRoLnJvdW5kKDEuMCAqIChqKzEpIC8gcmF0aW8pIC0gMTtcbiAgICAgIGlmIChzajIgPj0gaW1hZ2UuaGVpZ2h0KSBzajIgPSBpbWFnZS5oZWlnaHQgLSAxO1xuXG4gICAgICBsZXQgc3VtID0gMDtcbiAgICAgIGxldCBjb3VudCA9IDA7XG4gICAgICBmb3IgKGxldCBpaSA9IHNpMTsgaWkgPD0gc2kyOyBpaSsrKSB7XG4gICAgICAgIGZvciAobGV0IGpqID0gc2oxOyBqaiA8PSBzajI7IGpqKyspIHtcbiAgICAgICAgICBzdW0gKz0gKDEuMCAqIGltYWdlLmRhdGFbamogKiBpbWFnZS53aWR0aCArIGlpXSk7XG4gICAgICAgICAgY291bnQgKz0gMTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgICAgaW1hZ2VEYXRhW2ogKiB3aWR0aCArIGldID0gTWF0aC5mbG9vcihzdW0gLyBjb3VudCk7XG4gICAgfVxuICB9XG4gIHJldHVybiB7ZGF0YTogaW1hZ2VEYXRhLCB3aWR0aDogd2lkdGgsIGhlaWdodDogaGVpZ2h0fTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSB7XG4gIGRvd25zYW1wbGVCaWxpbmVhcixcbiAgdXBzYW1wbGVCaWxpbmVhcixcbiAgcmVzaXplLFxufVxuXG4iLCIvLyBUaGUgbW9kdWxlIGNhY2hlXG52YXIgX193ZWJwYWNrX21vZHVsZV9jYWNoZV9fID0ge307XG5cbi8vIFRoZSByZXF1aXJlIGZ1bmN0aW9uXG5mdW5jdGlvbiBfX3dlYnBhY2tfcmVxdWlyZV9fKG1vZHVsZUlkKSB7XG5cdC8vIENoZWNrIGlmIG1vZHVsZSBpcyBpbiBjYWNoZVxuXHR2YXIgY2FjaGVkTW9kdWxlID0gX193ZWJwYWNrX21vZHVsZV9jYWNoZV9fW21vZHVsZUlkXTtcblx0aWYgKGNhY2hlZE1vZHVsZSAhPT0gdW5kZWZpbmVkKSB7XG5cdFx0cmV0dXJuIGNhY2hlZE1vZHVsZS5leHBvcnRzO1xuXHR9XG5cdC8vIENyZWF0ZSBhIG5ldyBtb2R1bGUgKGFuZCBwdXQgaXQgaW50byB0aGUgY2FjaGUpXG5cdHZhciBtb2R1bGUgPSBfX3dlYnBhY2tfbW9kdWxlX2NhY2hlX19bbW9kdWxlSWRdID0ge1xuXHRcdC8vIG5vIG1vZHVsZS5pZCBuZWVkZWRcblx0XHQvLyBubyBtb2R1bGUubG9hZGVkIG5lZWRlZFxuXHRcdGV4cG9ydHM6IHt9XG5cdH07XG5cblx0Ly8gRXhlY3V0ZSB0aGUgbW9kdWxlIGZ1bmN0aW9uXG5cdF9fd2VicGFja19tb2R1bGVzX19bbW9kdWxlSWRdKG1vZHVsZSwgbW9kdWxlLmV4cG9ydHMsIF9fd2VicGFja19yZXF1aXJlX18pO1xuXG5cdC8vIFJldHVybiB0aGUgZXhwb3J0cyBvZiB0aGUgbW9kdWxlXG5cdHJldHVybiBtb2R1bGUuZXhwb3J0cztcbn1cblxuIiwiY29uc3Qge2V4dHJhY3R9ID0gcmVxdWlyZSgnLi90cmFja2VyL2V4dHJhY3QuanMnKTtcbmNvbnN0IHtidWlsZFRyYWNraW5nSW1hZ2VMaXN0fSA9IHJlcXVpcmUoJy4vaW1hZ2UtbGlzdC5qcycpO1xuXG5vbm1lc3NhZ2UgPSAobXNnKSA9PiB7XG4gIGNvbnN0IHtkYXRhfSA9IG1zZztcbiAgaWYgKGRhdGEudHlwZSA9PT0gJ2NvbXBpbGUnKSB7XG4gICAgLy9jb25zb2xlLmxvZyhcIndvcmtlciBjb21waWxlLi4uXCIpO1xuICAgIGNvbnN0IHt0YXJnZXRJbWFnZXN9ID0gZGF0YTtcbiAgICBjb25zdCBwZXJjZW50UGVySW1hZ2UgPSA1MC4wIC8gdGFyZ2V0SW1hZ2VzLmxlbmd0aDtcbiAgICBsZXQgcGVyY2VudCA9IDAuMDtcbiAgICBjb25zdCBsaXN0ID0gW107XG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0YXJnZXRJbWFnZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgIGNvbnN0IHRhcmdldEltYWdlID0gdGFyZ2V0SW1hZ2VzW2ldO1xuICAgICAgY29uc3QgaW1hZ2VMaXN0ID0gYnVpbGRUcmFja2luZ0ltYWdlTGlzdCh0YXJnZXRJbWFnZSk7XG4gICAgICBjb25zdCBwZXJjZW50UGVyQWN0aW9uID0gcGVyY2VudFBlckltYWdlIC8gaW1hZ2VMaXN0Lmxlbmd0aDtcblxuICAgICAgLy9jb25zb2xlLmxvZyhcImNvbXBpbGluZyB0cmFja2luZy4uLlwiLCBpKTtcbiAgICAgIGNvbnN0IHRyYWNraW5nRGF0YSA9IF9leHRyYWN0VHJhY2tpbmdGZWF0dXJlcyhpbWFnZUxpc3QsIChpbmRleCkgPT4ge1xuXHQvL2NvbnNvbGUubG9nKFwiZG9uZSB0cmFja2luZ1wiLCBpLCBpbmRleCk7XG5cdHBlcmNlbnQgKz0gcGVyY2VudFBlckFjdGlvblxuXHRwb3N0TWVzc2FnZSh7dHlwZTogJ3Byb2dyZXNzJywgcGVyY2VudH0pO1xuICAgICAgfSk7XG4gICAgICBsaXN0LnB1c2godHJhY2tpbmdEYXRhKTtcbiAgICB9XG4gICAgcG9zdE1lc3NhZ2Uoe1xuICAgICAgdHlwZTogJ2NvbXBpbGVEb25lJyxcbiAgICAgIGxpc3QsXG4gICAgfSk7XG4gIH1cbn07XG5cbmNvbnN0IF9leHRyYWN0VHJhY2tpbmdGZWF0dXJlcyA9IChpbWFnZUxpc3QsIGRvbmVDYWxsYmFjaykgPT4ge1xuICBjb25zdCBmZWF0dXJlU2V0cyA9IFtdO1xuICBmb3IgKGxldCBpID0gMDsgaSA8IGltYWdlTGlzdC5sZW5ndGg7IGkrKykge1xuICAgIGNvbnN0IGltYWdlID0gaW1hZ2VMaXN0W2ldO1xuICAgIGNvbnN0IHBvaW50cyA9IGV4dHJhY3QoaW1hZ2UpO1xuXG4gICAgY29uc3QgZmVhdHVyZVNldCA9IHtcbiAgICAgIGRhdGE6IGltYWdlLmRhdGEsXG4gICAgICBzY2FsZTogaW1hZ2Uuc2NhbGUsXG4gICAgICB3aWR0aDogaW1hZ2Uud2lkdGgsXG4gICAgICBoZWlnaHQ6IGltYWdlLmhlaWdodCxcbiAgICAgIHBvaW50cyxcbiAgICB9O1xuICAgIGZlYXR1cmVTZXRzLnB1c2goZmVhdHVyZVNldCk7XG5cbiAgICBkb25lQ2FsbGJhY2soaSk7XG4gIH1cbiAgcmV0dXJuIGZlYXR1cmVTZXRzO1xufVxuXG4iXSwibmFtZXMiOltdLCJzb3VyY2VSb290IjoiIn0=