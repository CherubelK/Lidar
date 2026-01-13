/**
 * 3D Trail Viewer using Three.js
 * Handles 3D scene setup, rendering, and controls
 */

class TrailViewer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.mesh = null;
        this.grid = null;
        this.axes = null;
        this.autoRotate = false;
        this.stats = {
            fps: 0,
            frameCount: 0,
            lastTime: performance.now()
        };

        this.init();
    }

    init() {
        // Create scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0a0a);
        this.scene.fog = new THREE.Fog(0x0a0a0a, 50, 200);

        // Create camera
        const aspect = this.canvas.clientWidth / this.canvas.clientHeight;
        this.camera = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000);
        this.camera.position.set(10, 10, 10);
        this.camera.lookAt(0, 0, 0);

        // Create renderer
        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(this.canvas.clientWidth, this.canvas.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

        // Create controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.screenSpacePanning = false;
        this.controls.minDistance = 1;
        this.controls.maxDistance = 100;
        this.controls.maxPolarAngle = Math.PI / 2;

        // Add lights
        this.setupLights();

        // Add grid and axes
        this.addGrid();
        this.addAxes();

        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());

        // Start animation loop
        this.animate();

        console.log('Trail Viewer initialized');
    }

    setupLights() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);

        // Directional light (sun)
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(10, 20, 10);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 2048;
        dirLight.shadow.mapSize.height = 2048;
        this.scene.add(dirLight);

        // Hemisphere light for better ambient
        const hemiLight = new THREE.HemisphereLight(0x8899bb, 0x334466, 0.3);
        this.scene.add(hemiLight);
    }

    addGrid() {
        this.grid = new THREE.GridHelper(50, 50, 0x667eea, 0x333333);
        this.scene.add(this.grid);
    }

    addAxes() {
        this.axes = new THREE.AxesHelper(5);
        this.scene.add(this.axes);
    }

    loadMesh(jsonPath) {
        return new Promise((resolve, reject) => {
            const loader = new THREE.FileLoader();

            loader.load(
                jsonPath,
                (data) => {
                    try {
                        const meshData = JSON.parse(data);
                        this.createMeshFromData(meshData);
                        resolve();
                    } catch (error) {
                        console.error('Error parsing mesh data:', error);
                        reject(error);
                    }
                },
                (xhr) => {
                    console.log((xhr.loaded / xhr.total * 100) + '% loaded');
                },
                (error) => {
                    console.error('Error loading mesh:', error);
                    reject(error);
                }
            );
        });
    }

    createMeshFromData(meshData) {
        // Remove existing mesh
        if (this.mesh) {
            this.scene.remove(this.mesh);
            this.mesh.geometry.dispose();
            this.mesh.material.dispose();
        }

        // Create geometry
        const geometry = new THREE.BufferGeometry();

        // Set positions
        const positions = new Float32Array(meshData.data.attributes.position.array);
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        // Set indices (faces)
        const indices = new Uint32Array(meshData.data.index.array);
        geometry.setIndex(new THREE.BufferAttribute(indices, 1));

        // Set normals if available
        if (meshData.data.attributes.normal) {
            const normals = new Float32Array(meshData.data.attributes.normal.array);
            geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
        } else {
            geometry.computeVertexNormals();
        }

        // Apply height-based coloring
        this.applyHeightColors(geometry);

        // Create material
        const material = new THREE.MeshPhongMaterial({
            vertexColors: true,
            side: THREE.DoubleSide,
            flatShading: false,
            shininess: 30
        });

        // Create mesh
        this.mesh = new THREE.Mesh(geometry, material);
        this.mesh.castShadow = true;
        this.mesh.receiveShadow = true;

        // Center the mesh
        geometry.computeBoundingBox();
        const center = new THREE.Vector3();
        geometry.boundingBox.getCenter(center);
        this.mesh.position.sub(center);

        this.scene.add(this.mesh);

        // Update camera to frame the object
        this.frameMesh();

        console.log(`Mesh loaded: ${positions.length / 3} vertices, ${indices.length / 3} faces`);
    }

    applyHeightColors(geometry) {
        const positions = geometry.attributes.position;
        const colors = new Float32Array(positions.count * 3);

        // Find min/max height
        let minZ = Infinity;
        let maxZ = -Infinity;

        for (let i = 0; i < positions.count; i++) {
            const z = positions.getZ(i);
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
        }

        // Apply colors based on height
        const colorLow = new THREE.Color(0x4a148c);   // Purple (low)
        const colorMid = new THREE.Color(0x2e7d32);   // Green (mid)
        const colorHigh = new THREE.Color(0xf57f17);  // Orange (high)

        for (let i = 0; i < positions.count; i++) {
            const z = positions.getZ(i);
            const t = (z - minZ) / (maxZ - minZ);

            let color;
            if (t < 0.5) {
                color = new THREE.Color().lerpColors(colorLow, colorMid, t * 2);
            } else {
                color = new THREE.Color().lerpColors(colorMid, colorHigh, (t - 0.5) * 2);
            }

            colors[i * 3] = color.r;
            colors[i * 3 + 1] = color.g;
            colors[i * 3 + 2] = color.b;
        }

        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    }

    frameMesh() {
        if (!this.mesh) return;

        const box = new THREE.Box3().setFromObject(this.mesh);
        const size = box.getSize(new THREE.Vector3());
        const center = box.getCenter(new THREE.Vector3());

        const maxDim = Math.max(size.x, size.y, size.z);
        const fov = this.camera.fov * (Math.PI / 180);
        let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
        cameraZ *= 1.5; // Add some padding

        this.camera.position.set(cameraZ, cameraZ * 0.7, cameraZ);
        this.camera.lookAt(center);
        this.camera.updateProjectionMatrix();

        this.controls.target.copy(center);
        this.controls.update();
    }

    setDisplayMode(mode) {
        if (!this.mesh) return;

        switch (mode) {
            case 'solid':
                this.mesh.material.wireframe = false;
                this.mesh.material.needsUpdate = true;
                break;
            case 'wireframe':
                this.mesh.material.wireframe = true;
                this.mesh.material.needsUpdate = true;
                break;
            case 'points':
                // TODO: Switch to points rendering
                console.log('Points mode not yet implemented');
                break;
        }
    }

    setColorMode(mode) {
        if (!this.mesh) return;

        const geometry = this.mesh.geometry;

        switch (mode) {
            case 'height':
                this.applyHeightColors(geometry);
                this.mesh.material.vertexColors = true;
                break;
            case 'solid':
                this.mesh.material.vertexColors = false;
                this.mesh.material.color = new THREE.Color(0x667eea);
                break;
            case 'normal':
                // Use normals for coloring
                this.applyNormalColors(geometry);
                this.mesh.material.vertexColors = true;
                break;
        }

        this.mesh.material.needsUpdate = true;
    }

    applyNormalColors(geometry) {
        const normals = geometry.attributes.normal;
        const colors = new Float32Array(normals.count * 3);

        for (let i = 0; i < normals.count; i++) {
            colors[i * 3] = (normals.getX(i) + 1) / 2;
            colors[i * 3 + 1] = (normals.getY(i) + 1) / 2;
            colors[i * 3 + 2] = (normals.getZ(i) + 1) / 2;
        }

        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    }

    setCameraView(view) {
        if (!this.mesh) return;

        const box = new THREE.Box3().setFromObject(this.mesh);
        const center = box.getCenter(new THREE.Vector3());
        const size = box.getSize(new THREE.Vector3());
        const maxDim = Math.max(size.x, size.y, size.z);
        const distance = maxDim * 1.5;

        switch (view) {
            case 'top':
                this.camera.position.set(center.x, center.y + distance, center.z);
                break;
            case 'side':
                this.camera.position.set(center.x + distance, center.y, center.z);
                break;
            case 'front':
                this.camera.position.set(center.x, center.y, center.z + distance);
                break;
            case 'perspective':
                this.camera.position.set(
                    center.x + distance,
                    center.y + distance * 0.7,
                    center.z + distance
                );
                break;
        }

        this.camera.lookAt(center);
        this.controls.target.copy(center);
        this.controls.update();
    }

    toggleGrid(show) {
        if (this.grid) {
            this.grid.visible = show;
        }
    }

    toggleAxes(show) {
        if (this.axes) {
            this.axes.visible = show;
        }
    }

    setAutoRotate(enabled) {
        this.autoRotate = enabled;
        this.controls.autoRotate = enabled;
        this.controls.autoRotateSpeed = 1.0;
    }

    takeScreenshot() {
        this.renderer.render(this.scene, this.camera);
        const dataURL = this.renderer.domElement.toDataURL('image/png');

        const link = document.createElement('a');
        link.download = `trail_screenshot_${Date.now()}.png`;
        link.href = dataURL;
        link.click();
    }

    onWindowResize() {
        this.camera.aspect = this.canvas.clientWidth / this.canvas.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.canvas.clientWidth, this.canvas.clientHeight);
    }

    updateStats() {
        this.stats.frameCount++;
        const currentTime = performance.now();
        const elapsed = currentTime - this.stats.lastTime;

        if (elapsed >= 1000) {
            this.stats.fps = Math.round((this.stats.frameCount * 1000) / elapsed);
            this.stats.frameCount = 0;
            this.stats.lastTime = currentTime;
        }
    }

    getStats() {
        return {
            fps: this.stats.fps,
            vertices: this.mesh ? this.mesh.geometry.attributes.position.count : 0,
            faces: this.mesh ? this.mesh.geometry.index.count / 3 : 0,
            cameraPosition: {
                x: this.camera.position.x.toFixed(2),
                y: this.camera.position.y.toFixed(2),
                z: this.camera.position.z.toFixed(2)
            }
        };
    }

    animate() {
        requestAnimationFrame(() => this.animate());

        this.controls.update();
        this.updateStats();
        this.renderer.render(this.scene, this.camera);
    }
}