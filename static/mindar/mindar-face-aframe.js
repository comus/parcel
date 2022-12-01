/******/ (() => { // webpackBootstrap
var __webpack_exports__ = {};
/*!**************************************!*\
  !*** ./mindar/face-target/aframe.js ***!
  \**************************************/
const {Controller, UI} = window.MINDAR.FACE;

const THREE = AFRAME.THREE;

AFRAME.registerSystem('mindar-face-system', {
  container: null,
  video: null,
  shouldFaceUser: true,
  lastHasFace: false,

  init: function() {
    this.anchorEntities = [];
    this.faceMeshEntities = [];
  },

  setup: function({uiLoading, uiScanning, uiError, filterMinCF, filterBeta}) {
    this.ui = new UI({uiLoading, uiScanning, uiError});
    this.filterMinCF = filterMinCF;
    this.filterBeta = filterBeta;
  },

  registerFaceMesh: function(el) {
    this.faceMeshEntities.push({el});
  },

  registerAnchor: function(el, anchorIndex) {
    this.anchorEntities.push({el: el, anchorIndex});
  },

  start: function() {
    this.ui.showLoading();

    this.container = this.el.sceneEl.parentNode;
    //this.__startVideo();
    this._startVideo();
  },

  stop: function() {
    this.pause();
    const tracks = this.video.srcObject.getTracks();
    tracks.forEach(function(track) {
      track.stop();
    });
    this.video.remove();
  },

  switchCamera: function() {
    this.shouldFaceUser = !this.shouldFaceUser;
    this.stop();
    this.start();
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

  // mock a video with an image
  __startVideo: function() {
    this.video = document.createElement("img");
    this.video.onload = async () => {
      this.video.videoWidth = this.video.width;
      this.video.videoHeight = this.video.height;

      await this._setupAR();
      this._processVideo();
      this.ui.hideLoading();
    }
    this.video.style.position = 'absolute'
    this.video.style.top = '0px'
    this.video.style.left = '0px'
    this.video.style.zIndex = '-2'
    this.video.src = "./assets/face1.jpeg";

    this.container.appendChild(this.video);
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
      this.el.emit("arError", {error: 'VIDEO_FAIL'});
      this.ui.showCompatibility();
      return;
    }

    navigator.mediaDevices.getUserMedia({audio: false, video: {
      facingMode: (this.shouldFaceUser? 'face': 'environment'),
    }}).then((stream) => {
      this.video.addEventListener( 'loadedmetadata', async () => {
        this.video.setAttribute('width', this.video.videoWidth);
        this.video.setAttribute('height', this.video.videoHeight);
        await this._setupAR();
	this._processVideo();
	this.ui.hideLoading();
      });
      this.video.srcObject = stream;
    }).catch((err) => {
      console.log("getUserMedia error", err);
      this.el.emit("arError", {error: 'VIDEO_FAIL'});
    });
  },

  _processVideo: function() {
    this.controller.onUpdate = ({hasFace, estimateResult}) => {

      if (hasFace && !this.lastHasFace) {
	this.el.emit("targetFound");
      }
      if (!hasFace && this.lastHasFace) {
	this.el.emit("targetLost");
      }
      this.lastHasFace = hasFace;

      if (hasFace) {
	const {faceMatrix} = estimateResult;
	for (let i = 0; i < this.anchorEntities.length; i++) {
	  const landmarkMatrix = this.controller.getLandmarkMatrix(this.anchorEntities[i].anchorIndex);
	  this.anchorEntities[i].el.updateVisibility(true);
	  this.anchorEntities[i].el.updateMatrix(landmarkMatrix);
	}

	for (let i = 0; i < this.faceMeshEntities.length; i++) {
	  this.faceMeshEntities[i].el.updateVisibility(true);
	  this.faceMeshEntities[i].el.updateMatrix(faceMatrix);
	}
      } else {
	for (let i = 0; i < this.anchorEntities.length; i++) {
	  this.anchorEntities[i].el.updateVisibility(false);
	}
	for (let i = 0; i < this.faceMeshEntities.length; i++) {
	  this.faceMeshEntities[i].el.updateVisibility(false);
	}
      }
    }
    this.controller.processVideo(this.video);
  },

  _setupAR: async function() {
    this.controller = new Controller({
      filterMinCF: this.filterMinCF,
      filterBeta: this.filterBeta,
    });
    this._resize();

    await this.controller.setup(this.video);
    await this.controller.dummyRun(this.video);
    const {fov, aspect, near, far} = this.controller.getCameraParams();

    const camera = new THREE.PerspectiveCamera();
    camera.fov = fov;
    camera.aspect = aspect;
    camera.near = near;
    camera.far = far;
    camera.updateProjectionMatrix();

    const cameraEle = this.container.getElementsByTagName("a-camera")[0];
    cameraEle.setObject3D('camera', camera);
    cameraEle.setAttribute('camera', 'active', true);

    for (let i = 0; i < this.faceMeshEntities.length; i++) {
      this.faceMeshEntities[i].el.addFaceMesh(this.controller.createThreeFaceGeometry(THREE));
    }

    this._resize();
    window.addEventListener('resize', this._resize.bind(this));
    this.el.emit("arReady");
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
    this.video.style.top = (-(vh - container.clientHeight) / 2) + "px";
    this.video.style.left = (-(vw - container.clientWidth) / 2) + "px";
    this.video.style.width = vw + "px";
    this.video.style.height = vh + "px";

    const sceneEl = container.getElementsByTagName("a-scene")[0];
    sceneEl.style.top = this.video.style.top;
    sceneEl.style.left = this.video.style.left;
    sceneEl.style.width = this.video.style.width;
    sceneEl.style.height = this.video.style.height;
  }
});

AFRAME.registerComponent('mindar-face', {
  dependencies: ['mindar-face-system'],

  schema: {
    autoStart: {type: 'boolean', default: true},
    faceOccluder: {type: 'boolean', default: true},
    uiLoading: {type: 'string', default: 'yes'},
    uiScanning: {type: 'string', default: 'yes'},
    uiError: {type: 'string', default: 'yes'},
    filterMinCF: {type: 'number', default: -1},
    filterBeta: {type: 'number', default: -1},
  },

  init: function() {
    const arSystem = this.el.sceneEl.systems['mindar-face-system'];

    if (this.data.faceOccluder) {
      const faceOccluderMeshEntity = document.createElement('a-entity');
      faceOccluderMeshEntity.setAttribute("mindar-face-default-face-occluder", true);
      this.el.sceneEl.appendChild(faceOccluderMeshEntity);
    }

    arSystem.setup({
      uiLoading: this.data.uiLoading,
      uiScanning: this.data.uiScanning,
      uiError: this.data.uiError,
      filterMinCF: this.data.filterMinCF === -1? null: this.data.filterMinCF,
      filterBeta: this.data.filterBeta === -1? null: this.data.filterBeta,
    });

    if (this.data.autoStart) {
      this.el.sceneEl.addEventListener('renderstart', () => {
        arSystem.start();
      });
    }
  },
});

AFRAME.registerComponent('mindar-face-target', {
  dependencies: ['mindar-face-system'],

  schema: {
    anchorIndex: {type: 'number'},
  },

  init: function() {
    const arSystem = this.el.sceneEl.systems['mindar-face-system'];
    arSystem.registerAnchor(this, this.data.anchorIndex);

    const root = this.el.object3D;
    root.visible = false;
    root.matrixAutoUpdate = false;
  },

  updateVisibility(visible) {
    this.el.object3D.visible = visible;
  },

  updateMatrix(matrix) {
    const root = this.el.object3D;
    root.matrix.set(...matrix);
  }
});

AFRAME.registerComponent('mindar-face-occluder', {
  init: function() {
    const root = this.el.object3D;
    this.el.addEventListener('model-loaded', () => {
      this.el.getObject3D('mesh').traverse((o) => {
	if (o.isMesh) {
	  const material = new THREE.MeshStandardMaterial({
	    colorWrite: false,
	  });
	  o.material = material;
	}
      });
    });
  },
});

AFRAME.registerComponent('mindar-face-default-face-occluder', {
  init: function() {
    const arSystem = this.el.sceneEl.systems['mindar-face-system'];
    arSystem.registerFaceMesh(this);

    const root = this.el.object3D;
    root.matrixAutoUpdate = false;
  },

  updateVisibility(visible) {
    this.el.object3D.visible = visible;
  },

  updateMatrix(matrix) {
    const root = this.el.object3D;
    root.matrix.set(...matrix);
  },

  addFaceMesh(faceGeometry) {
    const material = new THREE.MeshBasicMaterial({colorWrite: false});
    //const material = new THREE.MeshBasicMaterial({colorWrite: '#CCCCCC'});
    const mesh = new THREE.Mesh(faceGeometry, material);
    this.el.setObject3D('mesh', mesh);
  },
});

/******/ })()
;
//# sourceMappingURL=data:application/json;charset=utf-8;base64,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QTtBQUNBLElBQUk7QUFDSjtBQUNBO0FBQ0EsT0FBTztBQUNQLEtBQUs7QUFDTCxHQUFHO0FBQ0gsQ0FBQzs7QUFFRDtBQUNBO0FBQ0E7QUFDQTs7QUFFQTtBQUNBO0FBQ0EsR0FBRzs7QUFFSDtBQUNBO0FBQ0EsR0FBRzs7QUFFSDtBQUNBO0FBQ0E7QUFDQSxHQUFHOztBQUVIO0FBQ0Esa0RBQWtELGtCQUFrQjtBQUNwRSxvREFBb0Qsc0JBQXNCO0FBQzFFO0FBQ0E7QUFDQSxHQUFHO0FBQ0gsQ0FBQyIsInNvdXJjZXMiOlsid2VicGFjazovL3BhcmNlbC1hcHAvLi9taW5kYXIvZmFjZS10YXJnZXQvYWZyYW1lLmpzIl0sInNvdXJjZXNDb250ZW50IjpbImNvbnN0IHtDb250cm9sbGVyLCBVSX0gPSB3aW5kb3cuTUlOREFSLkZBQ0U7XG5cbmNvbnN0IFRIUkVFID0gQUZSQU1FLlRIUkVFO1xuXG5BRlJBTUUucmVnaXN0ZXJTeXN0ZW0oJ21pbmRhci1mYWNlLXN5c3RlbScsIHtcbiAgY29udGFpbmVyOiBudWxsLFxuICB2aWRlbzogbnVsbCxcbiAgc2hvdWxkRmFjZVVzZXI6IHRydWUsXG4gIGxhc3RIYXNGYWNlOiBmYWxzZSxcblxuICBpbml0OiBmdW5jdGlvbigpIHtcbiAgICB0aGlzLmFuY2hvckVudGl0aWVzID0gW107XG4gICAgdGhpcy5mYWNlTWVzaEVudGl0aWVzID0gW107XG4gIH0sXG5cbiAgc2V0dXA6IGZ1bmN0aW9uKHt1aUxvYWRpbmcsIHVpU2Nhbm5pbmcsIHVpRXJyb3IsIGZpbHRlck1pbkNGLCBmaWx0ZXJCZXRhfSkge1xuICAgIHRoaXMudWkgPSBuZXcgVUkoe3VpTG9hZGluZywgdWlTY2FubmluZywgdWlFcnJvcn0pO1xuICAgIHRoaXMuZmlsdGVyTWluQ0YgPSBmaWx0ZXJNaW5DRjtcbiAgICB0aGlzLmZpbHRlckJldGEgPSBmaWx0ZXJCZXRhO1xuICB9LFxuXG4gIHJlZ2lzdGVyRmFjZU1lc2g6IGZ1bmN0aW9uKGVsKSB7XG4gICAgdGhpcy5mYWNlTWVzaEVudGl0aWVzLnB1c2goe2VsfSk7XG4gIH0sXG5cbiAgcmVnaXN0ZXJBbmNob3I6IGZ1bmN0aW9uKGVsLCBhbmNob3JJbmRleCkge1xuICAgIHRoaXMuYW5jaG9yRW50aXRpZXMucHVzaCh7ZWw6IGVsLCBhbmNob3JJbmRleH0pO1xuICB9LFxuXG4gIHN0YXJ0OiBmdW5jdGlvbigpIHtcbiAgICB0aGlzLnVpLnNob3dMb2FkaW5nKCk7XG5cbiAgICB0aGlzLmNvbnRhaW5lciA9IHRoaXMuZWwuc2NlbmVFbC5wYXJlbnROb2RlO1xuICAgIC8vdGhpcy5fX3N0YXJ0VmlkZW8oKTtcbiAgICB0aGlzLl9zdGFydFZpZGVvKCk7XG4gIH0sXG5cbiAgc3RvcDogZnVuY3Rpb24oKSB7XG4gICAgdGhpcy5wYXVzZSgpO1xuICAgIGNvbnN0IHRyYWNrcyA9IHRoaXMudmlkZW8uc3JjT2JqZWN0LmdldFRyYWNrcygpO1xuICAgIHRyYWNrcy5mb3JFYWNoKGZ1bmN0aW9uKHRyYWNrKSB7XG4gICAgICB0cmFjay5zdG9wKCk7XG4gICAgfSk7XG4gICAgdGhpcy52aWRlby5yZW1vdmUoKTtcbiAgfSxcblxuICBzd2l0Y2hDYW1lcmE6IGZ1bmN0aW9uKCkge1xuICAgIHRoaXMuc2hvdWxkRmFjZVVzZXIgPSAhdGhpcy5zaG91bGRGYWNlVXNlcjtcbiAgICB0aGlzLnN0b3AoKTtcbiAgICB0aGlzLnN0YXJ0KCk7XG4gIH0sXG5cbiAgcGF1c2U6IGZ1bmN0aW9uKGtlZXBWaWRlbz1mYWxzZSkge1xuICAgIGlmICgha2VlcFZpZGVvKSB7XG4gICAgICB0aGlzLnZpZGVvLnBhdXNlKCk7XG4gICAgfVxuICAgIHRoaXMuY29udHJvbGxlci5zdG9wUHJvY2Vzc1ZpZGVvKCk7XG4gIH0sXG5cbiAgdW5wYXVzZTogZnVuY3Rpb24oKSB7XG4gICAgdGhpcy52aWRlby5wbGF5KCk7XG4gICAgdGhpcy5jb250cm9sbGVyLnByb2Nlc3NWaWRlbyh0aGlzLnZpZGVvKTtcbiAgfSxcblxuICAvLyBtb2NrIGEgdmlkZW8gd2l0aCBhbiBpbWFnZVxuICBfX3N0YXJ0VmlkZW86IGZ1bmN0aW9uKCkge1xuICAgIHRoaXMudmlkZW8gPSBkb2N1bWVudC5jcmVhdGVFbGVtZW50KFwiaW1nXCIpO1xuICAgIHRoaXMudmlkZW8ub25sb2FkID0gYXN5bmMgKCkgPT4ge1xuICAgICAgdGhpcy52aWRlby52aWRlb1dpZHRoID0gdGhpcy52aWRlby53aWR0aDtcbiAgICAgIHRoaXMudmlkZW8udmlkZW9IZWlnaHQgPSB0aGlzLnZpZGVvLmhlaWdodDtcblxuICAgICAgYXdhaXQgdGhpcy5fc2V0dXBBUigpO1xuICAgICAgdGhpcy5fcHJvY2Vzc1ZpZGVvKCk7XG4gICAgICB0aGlzLnVpLmhpZGVMb2FkaW5nKCk7XG4gICAgfVxuICAgIHRoaXMudmlkZW8uc3R5bGUucG9zaXRpb24gPSAnYWJzb2x1dGUnXG4gICAgdGhpcy52aWRlby5zdHlsZS50b3AgPSAnMHB4J1xuICAgIHRoaXMudmlkZW8uc3R5bGUubGVmdCA9ICcwcHgnXG4gICAgdGhpcy52aWRlby5zdHlsZS56SW5kZXggPSAnLTInXG4gICAgdGhpcy52aWRlby5zcmMgPSBcIi4vYXNzZXRzL2ZhY2UxLmpwZWdcIjtcblxuICAgIHRoaXMuY29udGFpbmVyLmFwcGVuZENoaWxkKHRoaXMudmlkZW8pO1xuICB9LFxuXG4gIF9zdGFydFZpZGVvOiBmdW5jdGlvbigpIHtcbiAgICB0aGlzLnZpZGVvID0gZG9jdW1lbnQuY3JlYXRlRWxlbWVudCgndmlkZW8nKTtcblxuICAgIHRoaXMudmlkZW8uc2V0QXR0cmlidXRlKCdhdXRvcGxheScsICcnKTtcbiAgICB0aGlzLnZpZGVvLnNldEF0dHJpYnV0ZSgnbXV0ZWQnLCAnJyk7XG4gICAgdGhpcy52aWRlby5zZXRBdHRyaWJ1dGUoJ3BsYXlzaW5saW5lJywgJycpO1xuICAgIHRoaXMudmlkZW8uc3R5bGUucG9zaXRpb24gPSAnYWJzb2x1dGUnXG4gICAgdGhpcy52aWRlby5zdHlsZS50b3AgPSAnMHB4J1xuICAgIHRoaXMudmlkZW8uc3R5bGUubGVmdCA9ICcwcHgnXG4gICAgdGhpcy52aWRlby5zdHlsZS56SW5kZXggPSAnLTInXG4gICAgdGhpcy5jb250YWluZXIuYXBwZW5kQ2hpbGQodGhpcy52aWRlbyk7XG5cbiAgICBpZiAoIW5hdmlnYXRvci5tZWRpYURldmljZXMgfHwgIW5hdmlnYXRvci5tZWRpYURldmljZXMuZ2V0VXNlck1lZGlhKSB7XG4gICAgICB0aGlzLmVsLmVtaXQoXCJhckVycm9yXCIsIHtlcnJvcjogJ1ZJREVPX0ZBSUwnfSk7XG4gICAgICB0aGlzLnVpLnNob3dDb21wYXRpYmlsaXR5KCk7XG4gICAgICByZXR1cm47XG4gICAgfVxuXG4gICAgbmF2aWdhdG9yLm1lZGlhRGV2aWNlcy5nZXRVc2VyTWVkaWEoe2F1ZGlvOiBmYWxzZSwgdmlkZW86IHtcbiAgICAgIGZhY2luZ01vZGU6ICh0aGlzLnNob3VsZEZhY2VVc2VyPyAnZmFjZSc6ICdlbnZpcm9ubWVudCcpLFxuICAgIH19KS50aGVuKChzdHJlYW0pID0+IHtcbiAgICAgIHRoaXMudmlkZW8uYWRkRXZlbnRMaXN0ZW5lciggJ2xvYWRlZG1ldGFkYXRhJywgYXN5bmMgKCkgPT4ge1xuICAgICAgICB0aGlzLnZpZGVvLnNldEF0dHJpYnV0ZSgnd2lkdGgnLCB0aGlzLnZpZGVvLnZpZGVvV2lkdGgpO1xuICAgICAgICB0aGlzLnZpZGVvLnNldEF0dHJpYnV0ZSgnaGVpZ2h0JywgdGhpcy52aWRlby52aWRlb0hlaWdodCk7XG4gICAgICAgIGF3YWl0IHRoaXMuX3NldHVwQVIoKTtcblx0dGhpcy5fcHJvY2Vzc1ZpZGVvKCk7XG5cdHRoaXMudWkuaGlkZUxvYWRpbmcoKTtcbiAgICAgIH0pO1xuICAgICAgdGhpcy52aWRlby5zcmNPYmplY3QgPSBzdHJlYW07XG4gICAgfSkuY2F0Y2goKGVycikgPT4ge1xuICAgICAgY29uc29sZS5sb2coXCJnZXRVc2VyTWVkaWEgZXJyb3JcIiwgZXJyKTtcbiAgICAgIHRoaXMuZWwuZW1pdChcImFyRXJyb3JcIiwge2Vycm9yOiAnVklERU9fRkFJTCd9KTtcbiAgICB9KTtcbiAgfSxcblxuICBfcHJvY2Vzc1ZpZGVvOiBmdW5jdGlvbigpIHtcbiAgICB0aGlzLmNvbnRyb2xsZXIub25VcGRhdGUgPSAoe2hhc0ZhY2UsIGVzdGltYXRlUmVzdWx0fSkgPT4ge1xuXG4gICAgICBpZiAoaGFzRmFjZSAmJiAhdGhpcy5sYXN0SGFzRmFjZSkge1xuXHR0aGlzLmVsLmVtaXQoXCJ0YXJnZXRGb3VuZFwiKTtcbiAgICAgIH1cbiAgICAgIGlmICghaGFzRmFjZSAmJiB0aGlzLmxhc3RIYXNGYWNlKSB7XG5cdHRoaXMuZWwuZW1pdChcInRhcmdldExvc3RcIik7XG4gICAgICB9XG4gICAgICB0aGlzLmxhc3RIYXNGYWNlID0gaGFzRmFjZTtcblxuICAgICAgaWYgKGhhc0ZhY2UpIHtcblx0Y29uc3Qge2ZhY2VNYXRyaXh9ID0gZXN0aW1hdGVSZXN1bHQ7XG5cdGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5hbmNob3JFbnRpdGllcy5sZW5ndGg7IGkrKykge1xuXHQgIGNvbnN0IGxhbmRtYXJrTWF0cml4ID0gdGhpcy5jb250cm9sbGVyLmdldExhbmRtYXJrTWF0cml4KHRoaXMuYW5jaG9yRW50aXRpZXNbaV0uYW5jaG9ySW5kZXgpO1xuXHQgIHRoaXMuYW5jaG9yRW50aXRpZXNbaV0uZWwudXBkYXRlVmlzaWJpbGl0eSh0cnVlKTtcblx0ICB0aGlzLmFuY2hvckVudGl0aWVzW2ldLmVsLnVwZGF0ZU1hdHJpeChsYW5kbWFya01hdHJpeCk7XG5cdH1cblxuXHRmb3IgKGxldCBpID0gMDsgaSA8IHRoaXMuZmFjZU1lc2hFbnRpdGllcy5sZW5ndGg7IGkrKykge1xuXHQgIHRoaXMuZmFjZU1lc2hFbnRpdGllc1tpXS5lbC51cGRhdGVWaXNpYmlsaXR5KHRydWUpO1xuXHQgIHRoaXMuZmFjZU1lc2hFbnRpdGllc1tpXS5lbC51cGRhdGVNYXRyaXgoZmFjZU1hdHJpeCk7XG5cdH1cbiAgICAgIH0gZWxzZSB7XG5cdGZvciAobGV0IGkgPSAwOyBpIDwgdGhpcy5hbmNob3JFbnRpdGllcy5sZW5ndGg7IGkrKykge1xuXHQgIHRoaXMuYW5jaG9yRW50aXRpZXNbaV0uZWwudXBkYXRlVmlzaWJpbGl0eShmYWxzZSk7XG5cdH1cblx0Zm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLmZhY2VNZXNoRW50aXRpZXMubGVuZ3RoOyBpKyspIHtcblx0ICB0aGlzLmZhY2VNZXNoRW50aXRpZXNbaV0uZWwudXBkYXRlVmlzaWJpbGl0eShmYWxzZSk7XG5cdH1cbiAgICAgIH1cbiAgICB9XG4gICAgdGhpcy5jb250cm9sbGVyLnByb2Nlc3NWaWRlbyh0aGlzLnZpZGVvKTtcbiAgfSxcblxuICBfc2V0dXBBUjogYXN5bmMgZnVuY3Rpb24oKSB7XG4gICAgdGhpcy5jb250cm9sbGVyID0gbmV3IENvbnRyb2xsZXIoe1xuICAgICAgZmlsdGVyTWluQ0Y6IHRoaXMuZmlsdGVyTWluQ0YsXG4gICAgICBmaWx0ZXJCZXRhOiB0aGlzLmZpbHRlckJldGEsXG4gICAgfSk7XG4gICAgdGhpcy5fcmVzaXplKCk7XG5cbiAgICBhd2FpdCB0aGlzLmNvbnRyb2xsZXIuc2V0dXAodGhpcy52aWRlbyk7XG4gICAgYXdhaXQgdGhpcy5jb250cm9sbGVyLmR1bW15UnVuKHRoaXMudmlkZW8pO1xuICAgIGNvbnN0IHtmb3YsIGFzcGVjdCwgbmVhciwgZmFyfSA9IHRoaXMuY29udHJvbGxlci5nZXRDYW1lcmFQYXJhbXMoKTtcblxuICAgIGNvbnN0IGNhbWVyYSA9IG5ldyBUSFJFRS5QZXJzcGVjdGl2ZUNhbWVyYSgpO1xuICAgIGNhbWVyYS5mb3YgPSBmb3Y7XG4gICAgY2FtZXJhLmFzcGVjdCA9IGFzcGVjdDtcbiAgICBjYW1lcmEubmVhciA9IG5lYXI7XG4gICAgY2FtZXJhLmZhciA9IGZhcjtcbiAgICBjYW1lcmEudXBkYXRlUHJvamVjdGlvbk1hdHJpeCgpO1xuXG4gICAgY29uc3QgY2FtZXJhRWxlID0gdGhpcy5jb250YWluZXIuZ2V0RWxlbWVudHNCeVRhZ05hbWUoXCJhLWNhbWVyYVwiKVswXTtcbiAgICBjYW1lcmFFbGUuc2V0T2JqZWN0M0QoJ2NhbWVyYScsIGNhbWVyYSk7XG4gICAgY2FtZXJhRWxlLnNldEF0dHJpYnV0ZSgnY2FtZXJhJywgJ2FjdGl2ZScsIHRydWUpO1xuXG4gICAgZm9yIChsZXQgaSA9IDA7IGkgPCB0aGlzLmZhY2VNZXNoRW50aXRpZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgIHRoaXMuZmFjZU1lc2hFbnRpdGllc1tpXS5lbC5hZGRGYWNlTWVzaCh0aGlzLmNvbnRyb2xsZXIuY3JlYXRlVGhyZWVGYWNlR2VvbWV0cnkoVEhSRUUpKTtcbiAgICB9XG5cbiAgICB0aGlzLl9yZXNpemUoKTtcbiAgICB3aW5kb3cuYWRkRXZlbnRMaXN0ZW5lcigncmVzaXplJywgdGhpcy5fcmVzaXplLmJpbmQodGhpcykpO1xuICAgIHRoaXMuZWwuZW1pdChcImFyUmVhZHlcIik7XG4gIH0sXG5cbiAgX3Jlc2l6ZTogZnVuY3Rpb24oKSB7XG4gICAgY29uc3QgdmlkZW8gPSB0aGlzLnZpZGVvO1xuICAgIGNvbnN0IGNvbnRhaW5lciA9IHRoaXMuY29udGFpbmVyO1xuICAgIGxldCB2dywgdmg7IC8vIGRpc3BsYXkgY3NzIHdpZHRoLCBoZWlnaHRcbiAgICBjb25zdCB2aWRlb1JhdGlvID0gdmlkZW8udmlkZW9XaWR0aCAvIHZpZGVvLnZpZGVvSGVpZ2h0O1xuICAgIGNvbnN0IGNvbnRhaW5lclJhdGlvID0gY29udGFpbmVyLmNsaWVudFdpZHRoIC8gY29udGFpbmVyLmNsaWVudEhlaWdodDtcbiAgICBpZiAodmlkZW9SYXRpbyA+IGNvbnRhaW5lclJhdGlvKSB7XG4gICAgICB2aCA9IGNvbnRhaW5lci5jbGllbnRIZWlnaHQ7XG4gICAgICB2dyA9IHZoICogdmlkZW9SYXRpbztcbiAgICB9IGVsc2Uge1xuICAgICAgdncgPSBjb250YWluZXIuY2xpZW50V2lkdGg7XG4gICAgICB2aCA9IHZ3IC8gdmlkZW9SYXRpbztcbiAgICB9XG4gICAgdGhpcy52aWRlby5zdHlsZS50b3AgPSAoLSh2aCAtIGNvbnRhaW5lci5jbGllbnRIZWlnaHQpIC8gMikgKyBcInB4XCI7XG4gICAgdGhpcy52aWRlby5zdHlsZS5sZWZ0ID0gKC0odncgLSBjb250YWluZXIuY2xpZW50V2lkdGgpIC8gMikgKyBcInB4XCI7XG4gICAgdGhpcy52aWRlby5zdHlsZS53aWR0aCA9IHZ3ICsgXCJweFwiO1xuICAgIHRoaXMudmlkZW8uc3R5bGUuaGVpZ2h0ID0gdmggKyBcInB4XCI7XG5cbiAgICBjb25zdCBzY2VuZUVsID0gY29udGFpbmVyLmdldEVsZW1lbnRzQnlUYWdOYW1lKFwiYS1zY2VuZVwiKVswXTtcbiAgICBzY2VuZUVsLnN0eWxlLnRvcCA9IHRoaXMudmlkZW8uc3R5bGUudG9wO1xuICAgIHNjZW5lRWwuc3R5bGUubGVmdCA9IHRoaXMudmlkZW8uc3R5bGUubGVmdDtcbiAgICBzY2VuZUVsLnN0eWxlLndpZHRoID0gdGhpcy52aWRlby5zdHlsZS53aWR0aDtcbiAgICBzY2VuZUVsLnN0eWxlLmhlaWdodCA9IHRoaXMudmlkZW8uc3R5bGUuaGVpZ2h0O1xuICB9XG59KTtcblxuQUZSQU1FLnJlZ2lzdGVyQ29tcG9uZW50KCdtaW5kYXItZmFjZScsIHtcbiAgZGVwZW5kZW5jaWVzOiBbJ21pbmRhci1mYWNlLXN5c3RlbSddLFxuXG4gIHNjaGVtYToge1xuICAgIGF1dG9TdGFydDoge3R5cGU6ICdib29sZWFuJywgZGVmYXVsdDogdHJ1ZX0sXG4gICAgZmFjZU9jY2x1ZGVyOiB7dHlwZTogJ2Jvb2xlYW4nLCBkZWZhdWx0OiB0cnVlfSxcbiAgICB1aUxvYWRpbmc6IHt0eXBlOiAnc3RyaW5nJywgZGVmYXVsdDogJ3llcyd9LFxuICAgIHVpU2Nhbm5pbmc6IHt0eXBlOiAnc3RyaW5nJywgZGVmYXVsdDogJ3llcyd9LFxuICAgIHVpRXJyb3I6IHt0eXBlOiAnc3RyaW5nJywgZGVmYXVsdDogJ3llcyd9LFxuICAgIGZpbHRlck1pbkNGOiB7dHlwZTogJ251bWJlcicsIGRlZmF1bHQ6IC0xfSxcbiAgICBmaWx0ZXJCZXRhOiB7dHlwZTogJ251bWJlcicsIGRlZmF1bHQ6IC0xfSxcbiAgfSxcblxuICBpbml0OiBmdW5jdGlvbigpIHtcbiAgICBjb25zdCBhclN5c3RlbSA9IHRoaXMuZWwuc2NlbmVFbC5zeXN0ZW1zWydtaW5kYXItZmFjZS1zeXN0ZW0nXTtcblxuICAgIGlmICh0aGlzLmRhdGEuZmFjZU9jY2x1ZGVyKSB7XG4gICAgICBjb25zdCBmYWNlT2NjbHVkZXJNZXNoRW50aXR5ID0gZG9jdW1lbnQuY3JlYXRlRWxlbWVudCgnYS1lbnRpdHknKTtcbiAgICAgIGZhY2VPY2NsdWRlck1lc2hFbnRpdHkuc2V0QXR0cmlidXRlKFwibWluZGFyLWZhY2UtZGVmYXVsdC1mYWNlLW9jY2x1ZGVyXCIsIHRydWUpO1xuICAgICAgdGhpcy5lbC5zY2VuZUVsLmFwcGVuZENoaWxkKGZhY2VPY2NsdWRlck1lc2hFbnRpdHkpO1xuICAgIH1cblxuICAgIGFyU3lzdGVtLnNldHVwKHtcbiAgICAgIHVpTG9hZGluZzogdGhpcy5kYXRhLnVpTG9hZGluZyxcbiAgICAgIHVpU2Nhbm5pbmc6IHRoaXMuZGF0YS51aVNjYW5uaW5nLFxuICAgICAgdWlFcnJvcjogdGhpcy5kYXRhLnVpRXJyb3IsXG4gICAgICBmaWx0ZXJNaW5DRjogdGhpcy5kYXRhLmZpbHRlck1pbkNGID09PSAtMT8gbnVsbDogdGhpcy5kYXRhLmZpbHRlck1pbkNGLFxuICAgICAgZmlsdGVyQmV0YTogdGhpcy5kYXRhLmZpbHRlckJldGEgPT09IC0xPyBudWxsOiB0aGlzLmRhdGEuZmlsdGVyQmV0YSxcbiAgICB9KTtcblxuICAgIGlmICh0aGlzLmRhdGEuYXV0b1N0YXJ0KSB7XG4gICAgICB0aGlzLmVsLnNjZW5lRWwuYWRkRXZlbnRMaXN0ZW5lcigncmVuZGVyc3RhcnQnLCAoKSA9PiB7XG4gICAgICAgIGFyU3lzdGVtLnN0YXJ0KCk7XG4gICAgICB9KTtcbiAgICB9XG4gIH0sXG59KTtcblxuQUZSQU1FLnJlZ2lzdGVyQ29tcG9uZW50KCdtaW5kYXItZmFjZS10YXJnZXQnLCB7XG4gIGRlcGVuZGVuY2llczogWydtaW5kYXItZmFjZS1zeXN0ZW0nXSxcblxuICBzY2hlbWE6IHtcbiAgICBhbmNob3JJbmRleDoge3R5cGU6ICdudW1iZXInfSxcbiAgfSxcblxuICBpbml0OiBmdW5jdGlvbigpIHtcbiAgICBjb25zdCBhclN5c3RlbSA9IHRoaXMuZWwuc2NlbmVFbC5zeXN0ZW1zWydtaW5kYXItZmFjZS1zeXN0ZW0nXTtcbiAgICBhclN5c3RlbS5yZWdpc3RlckFuY2hvcih0aGlzLCB0aGlzLmRhdGEuYW5jaG9ySW5kZXgpO1xuXG4gICAgY29uc3Qgcm9vdCA9IHRoaXMuZWwub2JqZWN0M0Q7XG4gICAgcm9vdC52aXNpYmxlID0gZmFsc2U7XG4gICAgcm9vdC5tYXRyaXhBdXRvVXBkYXRlID0gZmFsc2U7XG4gIH0sXG5cbiAgdXBkYXRlVmlzaWJpbGl0eSh2aXNpYmxlKSB7XG4gICAgdGhpcy5lbC5vYmplY3QzRC52aXNpYmxlID0gdmlzaWJsZTtcbiAgfSxcblxuICB1cGRhdGVNYXRyaXgobWF0cml4KSB7XG4gICAgY29uc3Qgcm9vdCA9IHRoaXMuZWwub2JqZWN0M0Q7XG4gICAgcm9vdC5tYXRyaXguc2V0KC4uLm1hdHJpeCk7XG4gIH1cbn0pO1xuXG5BRlJBTUUucmVnaXN0ZXJDb21wb25lbnQoJ21pbmRhci1mYWNlLW9jY2x1ZGVyJywge1xuICBpbml0OiBmdW5jdGlvbigpIHtcbiAgICBjb25zdCByb290ID0gdGhpcy5lbC5vYmplY3QzRDtcbiAgICB0aGlzLmVsLmFkZEV2ZW50TGlzdGVuZXIoJ21vZGVsLWxvYWRlZCcsICgpID0+IHtcbiAgICAgIHRoaXMuZWwuZ2V0T2JqZWN0M0QoJ21lc2gnKS50cmF2ZXJzZSgobykgPT4ge1xuXHRpZiAoby5pc01lc2gpIHtcblx0ICBjb25zdCBtYXRlcmlhbCA9IG5ldyBUSFJFRS5NZXNoU3RhbmRhcmRNYXRlcmlhbCh7XG5cdCAgICBjb2xvcldyaXRlOiBmYWxzZSxcblx0ICB9KTtcblx0ICBvLm1hdGVyaWFsID0gbWF0ZXJpYWw7XG5cdH1cbiAgICAgIH0pO1xuICAgIH0pO1xuICB9LFxufSk7XG5cbkFGUkFNRS5yZWdpc3RlckNvbXBvbmVudCgnbWluZGFyLWZhY2UtZGVmYXVsdC1mYWNlLW9jY2x1ZGVyJywge1xuICBpbml0OiBmdW5jdGlvbigpIHtcbiAgICBjb25zdCBhclN5c3RlbSA9IHRoaXMuZWwuc2NlbmVFbC5zeXN0ZW1zWydtaW5kYXItZmFjZS1zeXN0ZW0nXTtcbiAgICBhclN5c3RlbS5yZWdpc3RlckZhY2VNZXNoKHRoaXMpO1xuXG4gICAgY29uc3Qgcm9vdCA9IHRoaXMuZWwub2JqZWN0M0Q7XG4gICAgcm9vdC5tYXRyaXhBdXRvVXBkYXRlID0gZmFsc2U7XG4gIH0sXG5cbiAgdXBkYXRlVmlzaWJpbGl0eSh2aXNpYmxlKSB7XG4gICAgdGhpcy5lbC5vYmplY3QzRC52aXNpYmxlID0gdmlzaWJsZTtcbiAgfSxcblxuICB1cGRhdGVNYXRyaXgobWF0cml4KSB7XG4gICAgY29uc3Qgcm9vdCA9IHRoaXMuZWwub2JqZWN0M0Q7XG4gICAgcm9vdC5tYXRyaXguc2V0KC4uLm1hdHJpeCk7XG4gIH0sXG5cbiAgYWRkRmFjZU1lc2goZmFjZUdlb21ldHJ5KSB7XG4gICAgY29uc3QgbWF0ZXJpYWwgPSBuZXcgVEhSRUUuTWVzaEJhc2ljTWF0ZXJpYWwoe2NvbG9yV3JpdGU6IGZhbHNlfSk7XG4gICAgLy9jb25zdCBtYXRlcmlhbCA9IG5ldyBUSFJFRS5NZXNoQmFzaWNNYXRlcmlhbCh7Y29sb3JXcml0ZTogJyNDQ0NDQ0MnfSk7XG4gICAgY29uc3QgbWVzaCA9IG5ldyBUSFJFRS5NZXNoKGZhY2VHZW9tZXRyeSwgbWF0ZXJpYWwpO1xuICAgIHRoaXMuZWwuc2V0T2JqZWN0M0QoJ21lc2gnLCBtZXNoKTtcbiAgfSxcbn0pO1xuIl0sIm5hbWVzIjpbXSwic291cmNlUm9vdCI6IiJ9