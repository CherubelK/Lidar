/**
 * Unitree L2 LiDAR 3D Viewer
 * Modern viewer for LiDAR room scans
 */

class LiDARViewer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.mesh = null;
        this.currentModel = null;
        this.autoRotate = false;
        this.viewMode = 'solid';

        this.init();
        this.loadAvailableScans();
        this.setupEventListeners();
    }

    init() {
        const container = document.getElementById('canvas-container');

        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);
        this.scene.fog = new THREE.Fog(0x1a1a2e, 10, 50);

        // Camera
        const aspect = container.clientWidth / container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000);
        this.camera.position.set(5, 3, 5);
        this.camera.lookAt(0, 0, 0);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        container.appendChild(this.renderer.domElement);

        // Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.screenSpacePanning = false;
        this.controls.minDistance = 1;
        this.controls.maxDistance = 50;
        this.controls.maxPolarAngle = Math.PI;

        // Lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.camera.near = 0.1;
        directionalLight.shadow.camera.far = 50;
        this.scene.add(directionalLight);

        const fillLight = new THREE.DirectionalLight(0x667eea, 0.3);
        fillLight.position.set(-5, 3, -5);
        this.scene.add(fillLight);

        // Grid helper
        const gridHelper = new THREE.GridHelper(20, 20, 0x667eea, 0x444444);
        gridHelper.material.opacity = 0.2;
        gridHelper.material.transparent = true;
        this.scene.add(gridHelper);

        // Axes helper
        const axesHelper = new THREE.AxesHelper(5);
        this.scene.add(axesHelper);

        // Handle resize
        window.addEventListener('resize', () => this.onWindowResize(), false);

        // Start animation loop
        this.animate();

        // Hide loading
        setTimeout(() => {
            this.updateLoadingStatus('Viewer ready', '');
            document.getElementById('loading').classList.add('hidden');
        }, 500);
    }

    async loadAvailableScans() {
        try {
            const response = await fetch('models/');
            const text = await response.text();

            // Parse directory listing or use known scans
            const scans = this.parseDirectoryListing(text);

            const select = document.getElementById('scan-select');
            select.innerHTML = '<option value="">Select a scan...</option>';

            scans.forEach(scan => {
                const option = document.createElement('option');
                option.value = scan;
                option.textContent = scan.replace('.json', '').replace(/_/g, ' ');
                select.appendChild(option);
            });

            // Auto-load first scan if available
            if (scans.length > 0) {
                select.value = scans[0];
                this.loadModel(scans[0]);
            }
        } catch (error) {
            console.error('Error loading scan list:', error);
            // Fallback to known scans
            const knownScans = ['my_room.json', 'working_scan.json', 'demo_trail.json'];
            const select = document.getElementById('scan-select');
            knownScans.forEach(scan => {
                const option = document.createElement('option');
                option.value = scan;
                option.textContent = scan.replace('.json', '').replace(/_/g, ' ');
                select.appendChild(option);
            });
        }
    }

    parseDirectoryListing(html) {
        // Simple parser for directory listings
        const matches = html.match(/href="([^"]+\.json)"/g);
        if (!matches) return [];
        return matches.map(m => m.match(/href="([^"]+)"/)[1]);
    }

    async loadModel(filename) {
        if (!filename) return;

        this.updateLoadingStatus('Loading model...', filename);
        document.getElementById('loading').classList.remove('hidden');

        try {
            const response = await fetch(`models/${filename}`);
            const data = await response.json();

            this.currentModel = filename;
            this.updateStats(data);

            // Remove old mesh
            if (this.mesh) {
                this.scene.remove(this.mesh);
                this.mesh.geometry.dispose();
                this.mesh.material.dispose();
            }

            // Create geometry
            const geometry = new THREE.BufferGeometry();

            // Vertices
            const vertices = new Float32Array(data.vertices);
            geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));

            // Faces
            if (data.faces && data.faces.length > 0) {
                const indices = new Uint32Array(data.faces);
                geometry.setIndex(new THREE.BufferAttribute(indices, 1));
            }

            // Compute normals
            geometry.computeVertexNormals();

            // Compute bounding box for stats
            geometry.computeBoundingBox();
            const box = geometry.boundingBox;
            const size = new THREE.Vector3();
            box.getSize(size);

            // Create vertex colors based on height
            const colors = [];
            const positions = geometry.attributes.position.array;
            let minZ = Infinity, maxZ = -Infinity;

            for (let i = 2; i < positions.length; i += 3) {
                minZ = Math.min(minZ, positions[i]);
                maxZ = Math.max(maxZ, positions[i]);
            }

            for (let i = 2; i < positions.length; i += 3) {
                const z = positions[i];
                const normalized = (z - minZ) / (maxZ - minZ);

                // Color gradient from blue (low) to red (high)
                const color = new THREE.Color();
                color.setHSL(0.7 - normalized * 0.7, 0.8, 0.5);
                colors.push(color.r, color.g, color.b);
            }

            geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

            // Create material
            const material = new THREE.MeshPhongMaterial({
                vertexColors: true,
                side: THREE.DoubleSide,
                shininess: 30,
                flatShading: false
            });

            // Create mesh
            this.mesh = new THREE.Mesh(geometry, material);
            this.mesh.castShadow = true;
            this.mesh.receiveShadow = true;

            // Center and scale the mesh
            const center = new THREE.Vector3();
            box.getCenter(center);
            this.mesh.position.sub(center);

            // Add to scene
            this.scene.add(this.mesh);

            // Update camera to frame the object
            const maxDim = Math.max(size.x, size.y, size.z);
            const fov = this.camera.fov * (Math.PI / 180);
            let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
            cameraZ *= 1.5; // Add some padding

            this.camera.position.set(cameraZ, cameraZ * 0.6, cameraZ);
            this.camera.lookAt(0, 0, 0);
            this.controls.target.set(0, 0, 0);
            this.controls.update();

            // Update stats display
            document.getElementById('stat-dimensions').textContent =
                `${size.x.toFixed(2)}m × ${size.y.toFixed(2)}m × ${size.z.toFixed(2)}m`;
            document.getElementById('stat-name').textContent =
                filename.replace('.json', '').replace(/_/g, ' ');

            // Hide loading
            document.getElementById('loading').classList.add('hidden');

        } catch (error) {
            console.error('Error loading model:', error);
            this.updateLoadingStatus('Error loading model', error.message);
            setTimeout(() => {
                document.getElementById('loading').classList.add('hidden');
            }, 2000);
        }
    }

    updateStats(data) {
        const vertexCount = data.vertices.length / 3;
        const faceCount = data.faces ? data.faces.length / 3 : 0;

        document.getElementById('stat-vertices').textContent = vertexCount.toLocaleString();
        document.getElementById('stat-faces').textContent = faceCount.toLocaleString();
    }

    updateLoadingStatus(text, detail) {
        document.getElementById('loading-text').textContent = text;
        document.getElementById('loading-detail').textContent = detail;
    }

    setViewMode(mode) {
        if (!this.mesh) return;

        this.viewMode = mode;

        switch (mode) {
            case 'solid':
                this.mesh.material.wireframe = false;
                this.mesh.material.opacity = 1.0;
                this.mesh.material.transparent = false;
                break;

            case 'wireframe':
                this.mesh.material.wireframe = true;
                this.mesh.material.opacity = 1.0;
                this.mesh.material.transparent = false;
                break;

            case 'points':
                // Switch to points material
                const pointsMaterial = new THREE.PointsMaterial({
                    size: 0.05,
                    vertexColors: true
                });

                // Create points object
                const points = new THREE.Points(this.mesh.geometry, pointsMaterial);
                points.position.copy(this.mesh.position);

                this.scene.remove(this.mesh);
                this.mesh = points;
                this.scene.add(this.mesh);
                break;

            case 'transparent':
                this.mesh.material.wireframe = false;
                this.mesh.material.opacity = 0.7;
                this.mesh.material.transparent = true;
                break;
        }
    }

    resetView() {
        if (this.mesh) {
            const box = new THREE.Box3().setFromObject(this.mesh);
            const size = new THREE.Vector3();
            box.getSize(size);

            const maxDim = Math.max(size.x, size.y, size.z);
            const fov = this.camera.fov * (Math.PI / 180);
            let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
            cameraZ *= 1.5;

            this.camera.position.set(cameraZ, cameraZ * 0.6, cameraZ);
            this.camera.lookAt(0, 0, 0);
            this.controls.target.set(0, 0, 0);
            this.controls.update();
        }
    }

    setupEventListeners() {
        // Scan selection
        document.getElementById('scan-select').addEventListener('change', (e) => {
            if (e.target.value) {
                this.loadModel(e.target.value);
            }
        });

        // Reset view
        document.getElementById('reset-view').addEventListener('click', () => {
            this.resetView();
        });

        // Auto rotate
        document.getElementById('auto-rotate').addEventListener('click', (e) => {
            this.autoRotate = !this.autoRotate;
            this.controls.autoRotate = this.autoRotate;
            this.controls.autoRotateSpeed = 2.0;
            e.target.classList.toggle('active');
        });

        // View mode buttons
        document.querySelectorAll('.view-button').forEach(button => {
            button.addEventListener('click', (e) => {
                document.querySelectorAll('.view-button').forEach(b => b.classList.remove('active'));
                e.target.classList.add('active');
                this.setViewMode(e.target.dataset.mode);
            });
        });
    }

    onWindowResize() {
        const container = document.getElementById('canvas-container');
        this.camera.aspect = container.clientWidth / container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(container.clientWidth, container.clientHeight);
    }

    animate() {
        requestAnimationFrame(() => this.animate());

        // Update controls
        this.controls.update();

        // Render scene
        this.renderer.render(this.scene, this.camera);
    }
}

// Initialize viewer when page loads
window.addEventListener('DOMContentLoaded', () => {
    new LiDARViewer();
});