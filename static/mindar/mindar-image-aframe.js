/******/ (() => { // webpackBootstrap
var __webpack_exports__ = {};
/*!***************************************!*\
  !*** ./mindar/image-target/aframe.js ***!
  \***************************************/
const {Controller, UI} = window.MINDAR.IMAGE;

AFRAME.registerSystem('mindar-image-system', {
  container: null,
  video: null,
  processingImage: false,

  init: function() {
    this.anchorEntities = [];
  },

  tick: function() {
  },

  setup: function({imageTargetSrc, maxTrack, showStats, uiLoading, uiScanning, uiError, missTolerance, warmupTolerance, filterMinCF, filterBeta}) {
    this.imageTargetSrc = imageTargetSrc;
    this.maxTrack = maxTrack;
    this.filterMinCF = filterMinCF;
    this.filterBeta = filterBeta;
    this.missTolerance = missTolerance;
    this.warmupTolerance = warmupTolerance;
    this.showStats = showStats;
    this.ui = new UI({uiLoading, uiScanning, uiError});
  },

  registerAnchor: function(el, targetIndex) {
    this.anchorEntities.push({el: el, targetIndex: targetIndex});
  },

  start: function() {
    this.container = this.el.sceneEl.parentNode;

    if (this.showStats) {
      this.mainStats = new Stats();
      this.mainStats.showPanel( 0 ); // 0: fps, 1: ms, 2: mb, 3+: custom
      this.mainStats.domElement.style.cssText = 'position:absolute;top:0px;left:0px;z-index:999';
      this.container.appendChild(this.mainStats.domElement);
    }

    this.ui.showLoading();
    this._startVideo();
  },

  switchTarget: function(targetIndex) {
    this.controller.interestedTargetIndex = targetIndex;
  },

  stop: function() {
    this.pause();
    const tracks = this.video.srcObject.getTracks();
    tracks.forEach(function(track) {
      track.stop();
    });
    this.video.remove();
  },

  pause: function(keepVideo=false) {
    if (!keepVideo) {
      this.video.pause();
    }
    this.controller.stopProcessVideo();
  },

  unpause: function() {
    this.video.play();
    this.controller.processVideo(this.video);
  },

  _startVideo: function() {
    this.video = document.createElement('video');

    this.video.setAttribute('autoplay', '');
    this.video.setAttribute('muted', '');
    this.video.setAttribute('playsinline', '');
    this.video.style.position = 'absolute'
    this.video.style.top = '0px'
    this.video.style.left = '0px'
    this.video.style.zIndex = '-2'
    this.container.appendChild(this.video);

    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      // TODO: show unsupported error
      this.el.emit("arError", {error: 'VIDEO_FAIL'});
      this.ui.showCompatibility();
      return;
    }

    navigator.mediaDevices.getUserMedia({audio: false, video: {
      facingMode: 'environment',
    }}).then((stream) => {
      this.video.addEventListener( 'loadedmetadata', () => {
        //console.log("video ready...", this.video);
        this.video.setAttribute('width', this.video.videoWidth);
        this.video.setAttribute('height', this.video.videoHeight);
        this._startAR();
      });
      this.video.srcObject = stream;
    }).catch((err) => {
      console.log("getUserMedia error", err);
      this.el.emit("arError", {error: 'VIDEO_FAIL'});
    });
  },

  _startAR: async function() {
    const video = this.video;
    const container = this.container;

    this.controller = new Controller({
      inputWidth: video.videoWidth,
      inputHeight: video.videoHeight,
      maxTrack: this.maxTrack, 
      filterMinCF: this.filterMinCF,
      filterBeta: this.filterBeta,
      missTolerance: this.missTolerance,
      warmupTolerance: this.warmupTolerance,
      onUpdate: (data) => {
	if (data.type === 'processDone') {
	  if (this.mainStats) this.mainStats.update();
	}
	else if (data.type === 'updateMatrix') {
	  const {targetIndex, worldMatrix} = data;

	  for (let i = 0; i < this.anchorEntities.length; i++) {
	    if (this.anchorEntities[i].targetIndex === targetIndex) {
	      this.anchorEntities[i].el.updateWorldMatrix(worldMatrix, );
	      if (worldMatrix) {
		this.ui.hideScanning();
	      }
	    }
	  }
	}
      }
    });

    this._resize();
    window.addEventListener('resize', this._resize.bind(this));

    const {dimensions: imageTargetDimensions} = await this.controller.addImageTargets(this.imageTargetSrc);

    for (let i = 0; i < this.anchorEntities.length; i++) {
      const {el, targetIndex} = this.anchorEntities[i];
      if (targetIndex < imageTargetDimensions.length) {
        el.setupMarker(imageTargetDimensions[targetIndex]);
      }
    }

    await this.controller.dummyRun(this.video);
    this.el.emit("arReady");
    this.ui.hideLoading();
    this.ui.showScanning();

    this.controller.processVideo(this.video);
  },

  _resize: function() {
    const video = this.video;
    const container = this.container;

    let vw, vh; // display css width, height
    const videoRatio = video.videoWidth / video.videoHeight;
    const containerRatio = container.clientWidth / container.clientHeight;
    if (videoRatio > containerRatio) {
      vh = container.clientHeight;
      vw = vh * videoRatio;
    } else {
      vw = container.clientWidth;
      vh = vw / videoRatio;
    }

    const proj = this.controller.getProjectionMatrix();
    const fov = 2 * Math.atan(1/proj[5] / vh * container.clientHeight ) * 180 / Math.PI; // vertical fov
    const near = proj[14] / (proj[10] - 1.0);
    const far = proj[14] / (proj[10] + 1.0);
    const ratio = proj[5] / proj[0]; // (r-l) / (t-b)
    //console.log("loaded proj: ", proj, ". fov: ", fov, ". near: ", near, ". far: ", far, ". ratio: ", ratio);
    const newAspect = container.clientWidth / container.clientHeight;
    const cameraEle = container.getElementsByTagName("a-camera")[0];
    const camera = cameraEle.getObject3D('camera');
    camera.fov = fov;
    camera.aspect = newAspect;
    camera.near = near;
    camera.far = far;
    camera.updateProjectionMatrix();
    //const newCam = new AFRAME.THREE.PerspectiveCamera(fov, newRatio, near, far);
    //camera.getObject3D('camera').projectionMatrix = newCam.projectionMatrix;

    this.video.style.top = (-(vh - container.clientHeight) / 2) + "px";
    this.video.style.left = (-(vw - container.clientWidth) / 2) + "px";
    this.video.style.width = vw + "px";
    this.video.style.height = vh + "px";
  }
});

AFRAME.registerComponent('mindar-image', {
  dependencies: ['mindar-image-system'],

  schema: {
    imageTargetSrc: {type: 'string'},
    maxTrack: {type: 'int', default: 1},
    filterMinCF: {type: 'number', default: -1},
    filterBeta: {type: 'number', default: -1},
    missTolerance: {type: 'int', default: -1},
    warmupTolerance: {type: 'int', default: -1},
    showStats: {type: 'boolean', default: false},
    autoStart: {type: 'boolean', default: true},
    uiLoading: {type: 'string', default: 'yes'},
    uiScanning: {type: 'string', default: 'yes'},
    uiError: {type: 'string', default: 'yes'},
  },

  init: function() {
    const arSystem = this.el.sceneEl.systems['mindar-image-system'];

    arSystem.setup({
      imageTargetSrc: this.data.imageTargetSrc, 
      maxTrack: this.data.maxTrack,
      filterMinCF: this.data.filterMinCF === -1? null: this.data.filterMinCF,
      filterBeta: this.data.filterBeta === -1? null: this.data.filterBeta,
      missTolerance: this.data.missTolerance === -1? null: this.data.missTolerance,
      warmupTolerance: this.data.warmupTolerance === -1? null: this.data.warmupTolerance,
      showStats: this.data.showStats,
      uiLoading: this.data.uiLoading,
      uiScanning: this.data.uiScanning,
      uiError: this.data.uiError,
    });
    if (this.data.autoStart) {
      this.el.sceneEl.addEventListener('renderstart', () => {
        arSystem.start();
      });
    }
  }
});

AFRAME.registerComponent('mindar-image-target', {
  dependencies: ['mindar-image-system'],

  schema: {
    targetIndex: {type: 'number'},
  },

  postMatrix: null, // rescale the anchor to make width of 1 unit = physical width of card

  init: function() {
    const arSystem = this.el.sceneEl.systems['mindar-image-system'];
    arSystem.registerAnchor(this, this.data.targetIndex);

    const root = this.el.object3D;
    root.visible = false;
    root.matrixAutoUpdate = false;
  },

  setupMarker([markerWidth, markerHeight]) {
    const position = new AFRAME.THREE.Vector3();
    const quaternion = new AFRAME.THREE.Quaternion();
    const scale = new AFRAME.THREE.Vector3();
    position.x = markerWidth / 2;
    position.y = markerWidth / 2 + (markerHeight - markerWidth) / 2;
    scale.x = markerWidth;
    scale.y = markerWidth;
    scale.z = markerWidth;
    this.postMatrix = new AFRAME.THREE.Matrix4();
    this.postMatrix.compose(position, quaternion, scale);
  },

  updateWorldMatrix(worldMatrix) {
    if (!this.el.object3D.visible && worldMatrix !== null) {
      this.el.emit("targetFound");
    } else if (this.el.object3D.visible && worldMatrix === null) {
      this.el.emit("targetLost");
    }

    this.el.object3D.visible = worldMatrix !== null;
    if (worldMatrix === null) {
      return;
    }
    var m = new AFRAME.THREE.Matrix4();
    m.elements = worldMatrix;
    m.multiply(this.postMatrix);
    this.el.object3D.matrix = m;
  }
});

/******/ })()
;
//# sourceMappingURL=data:application/json;charset=utf-8;base64,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QUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSxDQUFDIiwic291cmNlcyI6WyJ3ZWJwYWNrOi8vcGFyY2VsLWFwcC8uL21pbmRhci9pbWFnZS10YXJnZXQvYWZyYW1lLmpzIl0sInNvdXJjZXNDb250ZW50IjpbImNvbnN0IHtDb250cm9sbGVyLCBVSX0gPSB3aW5kb3cuTUlOREFSLklNQUdFO1xuXG5BRlJBTUUucmVnaXN0ZXJTeXN0ZW0oJ21pbmRhci1pbWFnZS1zeXN0ZW0nLCB7XG4gIGNvbnRhaW5lcjogbnVsbCxcbiAgdmlkZW86IG51bGwsXG4gIHByb2Nlc3NpbmdJbWFnZTogZmFsc2UsXG5cbiAgaW5pdDogZnVuY3Rpb24oKSB7XG4gICAgdGhpcy5hbmNob3JFbnRpdGllcyA9IFtdO1xuICB9LFxuXG4gIHRpY2s6IGZ1bmN0aW9uKCkge1xuICB9LFxuXG4gIHNldHVwOiBmdW5jdGlvbih7aW1hZ2VUYXJnZXRTcmMsIG1heFRyYWNrLCBzaG93U3RhdHMsIHVpTG9hZGluZywgdWlTY2FubmluZywgdWlFcnJvciwgbWlzc1RvbGVyYW5jZSwgd2FybXVwVG9sZXJhbmNlLCBmaWx0ZXJNaW5DRiwgZmlsdGVyQmV0YX0pIHtcbiAgICB0aGlzLmltYWdlVGFyZ2V0U3JjID0gaW1hZ2VUYXJnZXRTcmM7XG4gICAgdGhpcy5tYXhUcmFjayA9IG1heFRyYWNrO1xuICAgIHRoaXMuZmlsdGVyTWluQ0YgPSBmaWx0ZXJNaW5DRjtcbiAgICB0aGlzLmZpbHRlckJldGEgPSBmaWx0ZXJCZXRhO1xuICAgIHRoaXMubWlzc1RvbGVyYW5jZSA9IG1pc3NUb2xlcmFuY2U7XG4gICAgdGhpcy53YXJtdXBUb2xlcmFuY2UgPSB3YXJtdXBUb2xlcmFuY2U7XG4gICAgdGhpcy5zaG93U3RhdHMgPSBzaG93U3RhdHM7XG4gICAgdGhpcy51aSA9IG5ldyBVSSh7dWlMb2FkaW5nLCB1aVNjYW5uaW5nLCB1aUVycm9yfSk7XG4gIH0sXG5cbiAgcmVnaXN0ZXJBbmNob3I6IGZ1bmN0aW9uKGVsLCB0YXJnZXRJbmRleCkge1xuICAgIHRoaXMuYW5jaG9yRW50aXRpZXMucHVzaCh7ZWw6IGVsLCB0YXJnZXRJbmRleDogdGFyZ2V0SW5kZXh9KTtcbiAgfSxcblxuICBzdGFydDogZnVuY3Rpb24oKSB7XG4gICAgdGhpcy5jb250YWluZXIgPSB0aGlzLmVsLnNjZW5lRWwucGFyZW50Tm9kZTtcblxuICAgIGlmICh0aGlzLnNob3dTdGF0cykge1xuICAgICAgdGhpcy5tYWluU3RhdHMgPSBuZXcgU3RhdHMoKTtcbiAgICAgIHRoaXMubWFpblN0YXRzLnNob3dQYW5lbCggMCApOyAvLyAwOiBmcHMsIDE6IG1zLCAyOiBtYiwgMys6IGN1c3RvbVxuICAgICAgdGhpcy5tYWluU3RhdHMuZG9tRWxlbWVudC5zdHlsZS5jc3NUZXh0ID0gJ3Bvc2l0aW9uOmFic29sdXRlO3RvcDowcHg7bGVmdDowcHg7ei1pbmRleDo5OTknO1xuICAgICAgdGhpcy5jb250YWluZXIuYXBwZW5kQ2hpbGQodGhpcy5tYWluU3RhdHMuZG9tRWxlbWVudCk7XG4gICAgfVxuXG4gICAgdGhpcy51aS5zaG93TG9hZGluZygpO1xuICAgIHRoaXMuX3N0YXJ0VmlkZW8oKTtcbiAgfSxcblxuICBzd2l0Y2hUYXJnZXQ6IGZ1bmN0aW9uKHRhcmdldEluZGV4KSB7XG4gICAgdGhpcy5jb250cm9sbGVyLmludGVyZXN0ZWRUYXJnZXRJbmRleCA9IHRhcmdldEluZGV4O1xuICB9LFxuXG4gIHN0b3A6IGZ1bmN0aW9uKCkge1xuICAgIHRoaXMucGF1c2UoKTtcbiAgICBjb25zdCB0cmFja3MgPSB0aGlzLnZpZGVvLnNyY09iamVjdC5nZXRUcmFja3MoKTtcbiAgICB0cmFja3MuZm9yRWFjaChmdW5jdGlvbih0cmFjaykge1xuICAgICAgdHJhY2suc3RvcCgpO1xuICAgIH0pO1xuICAgIHRoaXMudmlkZW8ucmVtb3ZlKCk7XG4gIH0sXG5cbiAgcGF1c2U6IGZ1bmN0aW9uKGtlZXBWaWRlbz1mYWxzZSkge1xuICAgIGlmICgha2VlcFZpZGVvKSB7XG4gICAgICB0aGlzLnZpZGVvLnBhdXNlKCk7XG4gICAgfVxuICAgIHRoaXMuY29udHJvbGxlci5zdG9wUHJvY2Vzc1ZpZGVvKCk7XG4gIH0sXG5cbiAgdW5wYXVzZTogZnVuY3Rpb24oKSB7XG4gICAgdGhpcy52aWRlby5wbGF5KCk7XG4gICAgdGhpcy5jb250cm9sbGVyLnByb2Nlc3NWaWRlbyh0aGlzLnZpZGVvKTtcbiAgfSxcblxuICBfc3RhcnRWaWRlbzogZnVuY3Rpb24oKSB7XG4gICAgdGhpcy52aWRlbyA9IGRvY3VtZW50LmNyZWF0ZUVsZW1lbnQoJ3ZpZGVvJyk7XG5cbiAgICB0aGlzLnZpZGVvLnNldEF0dHJpYnV0ZSgnYXV0b3BsYXknLCAnJyk7XG4gICAgdGhpcy52aWRlby5zZXRBdHRyaWJ1dGUoJ211dGVkJywgJycpO1xuICAgIHRoaXMudmlkZW8uc2V0QXR0cmlidXRlKCdwbGF5c2lubGluZScsICcnKTtcbiAgICB0aGlzLnZpZGVvLnN0eWxlLnBvc2l0aW9uID0gJ2Fic29sdXRlJ1xuICAgIHRoaXMudmlkZW8uc3R5bGUudG9wID0gJzBweCdcbiAgICB0aGlzLnZpZGVvLnN0eWxlLmxlZnQgPSAnMHB4J1xuICAgIHRoaXMudmlkZW8uc3R5bGUuekluZGV4ID0gJy0yJ1xuICAgIHRoaXMuY29udGFpbmVyLmFwcGVuZENoaWxkKHRoaXMudmlkZW8pO1xuXG4gICAgaWYgKCFuYXZpZ2F0b3IubWVkaWFEZXZpY2VzIHx8ICFuYXZpZ2F0b3IubWVkaWFEZXZpY2VzLmdldFVzZXJNZWRpYSkge1xuICAgICAgLy8gVE9ETzogc2hvdyB1bnN1cHBvcnRlZCBlcnJvclxuICAgICAgdGhpcy5lbC5lbWl0KFwiYXJFcnJvclwiLCB7ZXJyb3I6ICdWSURFT19GQUlMJ30pO1xuICAgICAgdGhpcy51aS5zaG93Q29tcGF0aWJpbGl0eSgpO1xuICAgICAgcmV0dXJuO1xuICAgIH1cblxuICAgIG5hdmlnYXRvci5tZWRpYURldmljZXMuZ2V0VXNlck1lZGlhKHthdWRpbzogZmFsc2UsIHZpZGVvOiB7XG4gICAgICBmYWNpbmdNb2RlOiAnZW52aXJvbm1lbnQnLFxuICAgIH19KS50aGVuKChzdHJlYW0pID0+IHtcbiAgICAgIHRoaXMudmlkZW8uYWRkRXZlbnRMaXN0ZW5lciggJ2xvYWRlZG1ldGFkYXRhJywgKCkgPT4ge1xuICAgICAgICAvL2NvbnNvbGUubG9nKFwidmlkZW8gcmVhZHkuLi5cIiwgdGhpcy52aWRlbyk7XG4gICAgICAgIHRoaXMudmlkZW8uc2V0QXR0cmlidXRlKCd3aWR0aCcsIHRoaXMudmlkZW8udmlkZW9XaWR0aCk7XG4gICAgICAgIHRoaXMudmlkZW8uc2V0QXR0cmlidXRlKCdoZWlnaHQnLCB0aGlzLnZpZGVvLnZpZGVvSGVpZ2h0KTtcbiAgICAgICAgdGhpcy5fc3RhcnRBUigpO1xuICAgICAgfSk7XG4gICAgICB0aGlzLnZpZGVvLnNyY09iamVjdCA9IHN0cmVhbTtcbiAgICB9KS5jYXRjaCgoZXJyKSA9PiB7XG4gICAgICBjb25zb2xlLmxvZyhcImdldFVzZXJNZWRpYSBlcnJvclwiLCBlcnIpO1xuICAgICAgdGhpcy5lbC5lbWl0KFwiYXJFcnJvclwiLCB7ZXJyb3I6ICdWSURFT19GQUlMJ30pO1xuICAgIH0pO1xuICB9LFxuXG4gIF9zdGFydEFSOiBhc3luYyBmdW5jdGlvbigpIHtcbiAgICBjb25zdCB2aWRlbyA9IHRoaXMudmlkZW87XG4gICAgY29uc3QgY29udGFpbmVyID0gdGhpcy5jb250YWluZXI7XG5cbiAgICB0aGlzLmNvbnRyb2xsZXIgPSBuZXcgQ29udHJvbGxlcih7XG4gICAgICBpbnB1dFdpZHRoOiB2aWRlby52aWRlb1dpZHRoLFxuICAgICAgaW5wdXRIZWlnaHQ6IHZpZGVvLnZpZGVvSGVpZ2h0LFxuICAgICAgbWF4VHJhY2s6IHRoaXMubWF4VHJhY2ssIFxuICAgICAgZmlsdGVyTWluQ0Y6IHRoaXMuZmlsdGVyTWluQ0YsXG4gICAgICBmaWx0ZXJCZXRhOiB0aGlzLmZpbHRlckJldGEsXG4gICAgICBtaXNzVG9sZXJhbmNlOiB0aGlzLm1pc3NUb2xlcmFuY2UsXG4gICAgICB3YXJtdXBUb2xlcmFuY2U6IHRoaXMud2FybXVwVG9sZXJhbmNlLFxuICAgICAgb25VcGRhdGU6IChkYXRhKSA9PiB7XG5cdGlmIChkYXRhLnR5cGUgPT09ICdwcm9jZXNzRG9uZScpIHtcblx0ICBpZiAodGhpcy5tYWluU3RhdHMpIHRoaXMubWFpblN0YXRzLnVwZGF0ZSgpO1xuXHR9XG5cdGVsc2UgaWYgKGRhdGEudHlwZSA9PT0gJ3VwZGF0ZU1hdHJpeCcpIHtcblx0ICBjb25zdCB7dGFyZ2V0SW5kZXgsIHdvcmxkTWF0cml4fSA9IGRhdGE7XG5cblx0ICBmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMuYW5jaG9yRW50aXRpZXMubGVuZ3RoOyBpKyspIHtcblx0ICAgIGlmICh0aGlzLmFuY2hvckVudGl0aWVzW2ldLnRhcmdldEluZGV4ID09PSB0YXJnZXRJbmRleCkge1xuXHQgICAgICB0aGlzLmFuY2hvckVudGl0aWVzW2ldLmVsLnVwZGF0ZVdvcmxkTWF0cml4KHdvcmxkTWF0cml4LCApO1xuXHQgICAgICBpZiAod29ybGRNYXRyaXgpIHtcblx0XHR0aGlzLnVpLmhpZGVTY2FubmluZygpO1xuXHQgICAgICB9XG5cdCAgICB9XG5cdCAgfVxuXHR9XG4gICAgICB9XG4gICAgfSk7XG5cbiAgICB0aGlzLl9yZXNpemUoKTtcbiAgICB3aW5kb3cuYWRkRXZlbnRMaXN0ZW5lcigncmVzaXplJywgdGhpcy5fcmVzaXplLmJpbmQodGhpcykpO1xuXG4gICAgY29uc3Qge2RpbWVuc2lvbnM6IGltYWdlVGFyZ2V0RGltZW5zaW9uc30gPSBhd2FpdCB0aGlzLmNvbnRyb2xsZXIuYWRkSW1hZ2VUYXJnZXRzKHRoaXMuaW1hZ2VUYXJnZXRTcmMpO1xuXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLmFuY2hvckVudGl0aWVzLmxlbmd0aDsgaSsrKSB7XG4gICAgICBjb25zdCB7ZWwsIHRhcmdldEluZGV4fSA9IHRoaXMuYW5jaG9yRW50aXRpZXNbaV07XG4gICAgICBpZiAodGFyZ2V0SW5kZXggPCBpbWFnZVRhcmdldERpbWVuc2lvbnMubGVuZ3RoKSB7XG4gICAgICAgIGVsLnNldHVwTWFya2VyKGltYWdlVGFyZ2V0RGltZW5zaW9uc1t0YXJnZXRJbmRleF0pO1xuICAgICAgfVxuICAgIH1cblxuICAgIGF3YWl0IHRoaXMuY29udHJvbGxlci5kdW1teVJ1bih0aGlzLnZpZGVvKTtcbiAgICB0aGlzLmVsLmVtaXQoXCJhclJlYWR5XCIpO1xuICAgIHRoaXMudWkuaGlkZUxvYWRpbmcoKTtcbiAgICB0aGlzLnVpLnNob3dTY2FubmluZygpO1xuXG4gICAgdGhpcy5jb250cm9sbGVyLnByb2Nlc3NWaWRlbyh0aGlzLnZpZGVvKTtcbiAgfSxcblxuICBfcmVzaXplOiBmdW5jdGlvbigpIHtcbiAgICBjb25zdCB2aWRlbyA9IHRoaXMudmlkZW87XG4gICAgY29uc3QgY29udGFpbmVyID0gdGhpcy5jb250YWluZXI7XG5cbiAgICBsZXQgdncsIHZoOyAvLyBkaXNwbGF5IGNzcyB3aWR0aCwgaGVpZ2h0XG4gICAgY29uc3QgdmlkZW9SYXRpbyA9IHZpZGVvLnZpZGVvV2lkdGggLyB2aWRlby52aWRlb0hlaWdodDtcbiAgICBjb25zdCBjb250YWluZXJSYXRpbyA9IGNvbnRhaW5lci5jbGllbnRXaWR0aCAvIGNvbnRhaW5lci5jbGllbnRIZWlnaHQ7XG4gICAgaWYgKHZpZGVvUmF0aW8gPiBjb250YWluZXJSYXRpbykge1xuICAgICAgdmggPSBjb250YWluZXIuY2xpZW50SGVpZ2h0O1xuICAgICAgdncgPSB2aCAqIHZpZGVvUmF0aW87XG4gICAgfSBlbHNlIHtcbiAgICAgIHZ3ID0gY29udGFpbmVyLmNsaWVudFdpZHRoO1xuICAgICAgdmggPSB2dyAvIHZpZGVvUmF0aW87XG4gICAgfVxuXG4gICAgY29uc3QgcHJvaiA9IHRoaXMuY29udHJvbGxlci5nZXRQcm9qZWN0aW9uTWF0cml4KCk7XG4gICAgY29uc3QgZm92ID0gMiAqIE1hdGguYXRhbigxL3Byb2pbNV0gLyB2aCAqIGNvbnRhaW5lci5jbGllbnRIZWlnaHQgKSAqIDE4MCAvIE1hdGguUEk7IC8vIHZlcnRpY2FsIGZvdlxuICAgIGNvbnN0IG5lYXIgPSBwcm9qWzE0XSAvIChwcm9qWzEwXSAtIDEuMCk7XG4gICAgY29uc3QgZmFyID0gcHJvalsxNF0gLyAocHJvalsxMF0gKyAxLjApO1xuICAgIGNvbnN0IHJhdGlvID0gcHJvals1XSAvIHByb2pbMF07IC8vIChyLWwpIC8gKHQtYilcbiAgICAvL2NvbnNvbGUubG9nKFwibG9hZGVkIHByb2o6IFwiLCBwcm9qLCBcIi4gZm92OiBcIiwgZm92LCBcIi4gbmVhcjogXCIsIG5lYXIsIFwiLiBmYXI6IFwiLCBmYXIsIFwiLiByYXRpbzogXCIsIHJhdGlvKTtcbiAgICBjb25zdCBuZXdBc3BlY3QgPSBjb250YWluZXIuY2xpZW50V2lkdGggLyBjb250YWluZXIuY2xpZW50SGVpZ2h0O1xuICAgIGNvbnN0IGNhbWVyYUVsZSA9IGNvbnRhaW5lci5nZXRFbGVtZW50c0J5VGFnTmFtZShcImEtY2FtZXJhXCIpWzBdO1xuICAgIGNvbnN0IGNhbWVyYSA9IGNhbWVyYUVsZS5nZXRPYmplY3QzRCgnY2FtZXJhJyk7XG4gICAgY2FtZXJhLmZvdiA9IGZvdjtcbiAgICBjYW1lcmEuYXNwZWN0ID0gbmV3QXNwZWN0O1xuICAgIGNhbWVyYS5uZWFyID0gbmVhcjtcbiAgICBjYW1lcmEuZmFyID0gZmFyO1xuICAgIGNhbWVyYS51cGRhdGVQcm9qZWN0aW9uTWF0cml4KCk7XG4gICAgLy9jb25zdCBuZXdDYW0gPSBuZXcgQUZSQU1FLlRIUkVFLlBlcnNwZWN0aXZlQ2FtZXJhKGZvdiwgbmV3UmF0aW8sIG5lYXIsIGZhcik7XG4gICAgLy9jYW1lcmEuZ2V0T2JqZWN0M0QoJ2NhbWVyYScpLnByb2plY3Rpb25NYXRyaXggPSBuZXdDYW0ucHJvamVjdGlvbk1hdHJpeDtcblxuICAgIHRoaXMudmlkZW8uc3R5bGUudG9wID0gKC0odmggLSBjb250YWluZXIuY2xpZW50SGVpZ2h0KSAvIDIpICsgXCJweFwiO1xuICAgIHRoaXMudmlkZW8uc3R5bGUubGVmdCA9ICgtKHZ3IC0gY29udGFpbmVyLmNsaWVudFdpZHRoKSAvIDIpICsgXCJweFwiO1xuICAgIHRoaXMudmlkZW8uc3R5bGUud2lkdGggPSB2dyArIFwicHhcIjtcbiAgICB0aGlzLnZpZGVvLnN0eWxlLmhlaWdodCA9IHZoICsgXCJweFwiO1xuICB9XG59KTtcblxuQUZSQU1FLnJlZ2lzdGVyQ29tcG9uZW50KCdtaW5kYXItaW1hZ2UnLCB7XG4gIGRlcGVuZGVuY2llczogWydtaW5kYXItaW1hZ2Utc3lzdGVtJ10sXG5cbiAgc2NoZW1hOiB7XG4gICAgaW1hZ2VUYXJnZXRTcmM6IHt0eXBlOiAnc3RyaW5nJ30sXG4gICAgbWF4VHJhY2s6IHt0eXBlOiAnaW50JywgZGVmYXVsdDogMX0sXG4gICAgZmlsdGVyTWluQ0Y6IHt0eXBlOiAnbnVtYmVyJywgZGVmYXVsdDogLTF9LFxuICAgIGZpbHRlckJldGE6IHt0eXBlOiAnbnVtYmVyJywgZGVmYXVsdDogLTF9LFxuICAgIG1pc3NUb2xlcmFuY2U6IHt0eXBlOiAnaW50JywgZGVmYXVsdDogLTF9LFxuICAgIHdhcm11cFRvbGVyYW5jZToge3R5cGU6ICdpbnQnLCBkZWZhdWx0OiAtMX0sXG4gICAgc2hvd1N0YXRzOiB7dHlwZTogJ2Jvb2xlYW4nLCBkZWZhdWx0OiBmYWxzZX0sXG4gICAgYXV0b1N0YXJ0OiB7dHlwZTogJ2Jvb2xlYW4nLCBkZWZhdWx0OiB0cnVlfSxcbiAgICB1aUxvYWRpbmc6IHt0eXBlOiAnc3RyaW5nJywgZGVmYXVsdDogJ3llcyd9LFxuICAgIHVpU2Nhbm5pbmc6IHt0eXBlOiAnc3RyaW5nJywgZGVmYXVsdDogJ3llcyd9LFxuICAgIHVpRXJyb3I6IHt0eXBlOiAnc3RyaW5nJywgZGVmYXVsdDogJ3llcyd9LFxuICB9LFxuXG4gIGluaXQ6IGZ1bmN0aW9uKCkge1xuICAgIGNvbnN0IGFyU3lzdGVtID0gdGhpcy5lbC5zY2VuZUVsLnN5c3RlbXNbJ21pbmRhci1pbWFnZS1zeXN0ZW0nXTtcblxuICAgIGFyU3lzdGVtLnNldHVwKHtcbiAgICAgIGltYWdlVGFyZ2V0U3JjOiB0aGlzLmRhdGEuaW1hZ2VUYXJnZXRTcmMsIFxuICAgICAgbWF4VHJhY2s6IHRoaXMuZGF0YS5tYXhUcmFjayxcbiAgICAgIGZpbHRlck1pbkNGOiB0aGlzLmRhdGEuZmlsdGVyTWluQ0YgPT09IC0xPyBudWxsOiB0aGlzLmRhdGEuZmlsdGVyTWluQ0YsXG4gICAgICBmaWx0ZXJCZXRhOiB0aGlzLmRhdGEuZmlsdGVyQmV0YSA9PT0gLTE/IG51bGw6IHRoaXMuZGF0YS5maWx0ZXJCZXRhLFxuICAgICAgbWlzc1RvbGVyYW5jZTogdGhpcy5kYXRhLm1pc3NUb2xlcmFuY2UgPT09IC0xPyBudWxsOiB0aGlzLmRhdGEubWlzc1RvbGVyYW5jZSxcbiAgICAgIHdhcm11cFRvbGVyYW5jZTogdGhpcy5kYXRhLndhcm11cFRvbGVyYW5jZSA9PT0gLTE/IG51bGw6IHRoaXMuZGF0YS53YXJtdXBUb2xlcmFuY2UsXG4gICAgICBzaG93U3RhdHM6IHRoaXMuZGF0YS5zaG93U3RhdHMsXG4gICAgICB1aUxvYWRpbmc6IHRoaXMuZGF0YS51aUxvYWRpbmcsXG4gICAgICB1aVNjYW5uaW5nOiB0aGlzLmRhdGEudWlTY2FubmluZyxcbiAgICAgIHVpRXJyb3I6IHRoaXMuZGF0YS51aUVycm9yLFxuICAgIH0pO1xuICAgIGlmICh0aGlzLmRhdGEuYXV0b1N0YXJ0KSB7XG4gICAgICB0aGlzLmVsLnNjZW5lRWwuYWRkRXZlbnRMaXN0ZW5lcigncmVuZGVyc3RhcnQnLCAoKSA9PiB7XG4gICAgICAgIGFyU3lzdGVtLnN0YXJ0KCk7XG4gICAgICB9KTtcbiAgICB9XG4gIH1cbn0pO1xuXG5BRlJBTUUucmVnaXN0ZXJDb21wb25lbnQoJ21pbmRhci1pbWFnZS10YXJnZXQnLCB7XG4gIGRlcGVuZGVuY2llczogWydtaW5kYXItaW1hZ2Utc3lzdGVtJ10sXG5cbiAgc2NoZW1hOiB7XG4gICAgdGFyZ2V0SW5kZXg6IHt0eXBlOiAnbnVtYmVyJ30sXG4gIH0sXG5cbiAgcG9zdE1hdHJpeDogbnVsbCwgLy8gcmVzY2FsZSB0aGUgYW5jaG9yIHRvIG1ha2Ugd2lkdGggb2YgMSB1bml0ID0gcGh5c2ljYWwgd2lkdGggb2YgY2FyZFxuXG4gIGluaXQ6IGZ1bmN0aW9uKCkge1xuICAgIGNvbnN0IGFyU3lzdGVtID0gdGhpcy5lbC5zY2VuZUVsLnN5c3RlbXNbJ21pbmRhci1pbWFnZS1zeXN0ZW0nXTtcbiAgICBhclN5c3RlbS5yZWdpc3RlckFuY2hvcih0aGlzLCB0aGlzLmRhdGEudGFyZ2V0SW5kZXgpO1xuXG4gICAgY29uc3Qgcm9vdCA9IHRoaXMuZWwub2JqZWN0M0Q7XG4gICAgcm9vdC52aXNpYmxlID0gZmFsc2U7XG4gICAgcm9vdC5tYXRyaXhBdXRvVXBkYXRlID0gZmFsc2U7XG4gIH0sXG5cbiAgc2V0dXBNYXJrZXIoW21hcmtlcldpZHRoLCBtYXJrZXJIZWlnaHRdKSB7XG4gICAgY29uc3QgcG9zaXRpb24gPSBuZXcgQUZSQU1FLlRIUkVFLlZlY3RvcjMoKTtcbiAgICBjb25zdCBxdWF0ZXJuaW9uID0gbmV3IEFGUkFNRS5USFJFRS5RdWF0ZXJuaW9uKCk7XG4gICAgY29uc3Qgc2NhbGUgPSBuZXcgQUZSQU1FLlRIUkVFLlZlY3RvcjMoKTtcbiAgICBwb3NpdGlvbi54ID0gbWFya2VyV2lkdGggLyAyO1xuICAgIHBvc2l0aW9uLnkgPSBtYXJrZXJXaWR0aCAvIDIgKyAobWFya2VySGVpZ2h0IC0gbWFya2VyV2lkdGgpIC8gMjtcbiAgICBzY2FsZS54ID0gbWFya2VyV2lkdGg7XG4gICAgc2NhbGUueSA9IG1hcmtlcldpZHRoO1xuICAgIHNjYWxlLnogPSBtYXJrZXJXaWR0aDtcbiAgICB0aGlzLnBvc3RNYXRyaXggPSBuZXcgQUZSQU1FLlRIUkVFLk1hdHJpeDQoKTtcbiAgICB0aGlzLnBvc3RNYXRyaXguY29tcG9zZShwb3NpdGlvbiwgcXVhdGVybmlvbiwgc2NhbGUpO1xuICB9LFxuXG4gIHVwZGF0ZVdvcmxkTWF0cml4KHdvcmxkTWF0cml4KSB7XG4gICAgaWYgKCF0aGlzLmVsLm9iamVjdDNELnZpc2libGUgJiYgd29ybGRNYXRyaXggIT09IG51bGwpIHtcbiAgICAgIHRoaXMuZWwuZW1pdChcInRhcmdldEZvdW5kXCIpO1xuICAgIH0gZWxzZSBpZiAodGhpcy5lbC5vYmplY3QzRC52aXNpYmxlICYmIHdvcmxkTWF0cml4ID09PSBudWxsKSB7XG4gICAgICB0aGlzLmVsLmVtaXQoXCJ0YXJnZXRMb3N0XCIpO1xuICAgIH1cblxuICAgIHRoaXMuZWwub2JqZWN0M0QudmlzaWJsZSA9IHdvcmxkTWF0cml4ICE9PSBudWxsO1xuICAgIGlmICh3b3JsZE1hdHJpeCA9PT0gbnVsbCkge1xuICAgICAgcmV0dXJuO1xuICAgIH1cbiAgICB2YXIgbSA9IG5ldyBBRlJBTUUuVEhSRUUuTWF0cml4NCgpO1xuICAgIG0uZWxlbWVudHMgPSB3b3JsZE1hdHJpeDtcbiAgICBtLm11bHRpcGx5KHRoaXMucG9zdE1hdHJpeCk7XG4gICAgdGhpcy5lbC5vYmplY3QzRC5tYXRyaXggPSBtO1xuICB9XG59KTtcbiJdLCJuYW1lcyI6W10sInNvdXJjZVJvb3QiOiIifQ==