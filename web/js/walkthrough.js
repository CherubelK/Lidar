/**
 * Walkthrough Mode
 * First-person flythrough of a captured scan using PointerLockControls.
 *
 * The captured data uses its 3rd component (data.z) as the vertical axis
 * (the same convention lidar-viewer.js uses when coloring by height), but
 * Three.js's PointerLockControls assumes Y is up and strafes in the X/Z
 * plane. Points are remapped on load: three.x = data.x, three.y = data.z,
 * three.z = data.y.
 *
 * There's no solid floor mesh to raycast against in a typical point-cloud
 * export, so this is a free-fly walkthrough (WASD + mouse look, Space/Shift
 * to rise/descend) rather than gravity-and-collision walking -- closer to
 * a "ghost"/spectator mode than a physics-based character controller.
 * Horizontal and vertical movement is clamped to the scan's bounding box
 * (with margin) so you can't fly off into empty space indefinitely.
 */
class WalkthroughViewer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.object = null;
        this.bounds = null;

        this.move = { forward: false, back: false, left: false, right: false, up: false, down: false, sprint: false };
        this.velocity = new THREE.Vector3();
        this.clock = new THREE.Clock();

        this.baseSpeed = 2.5;   // m/s
        this.sprintMultiplier = 2.5;

        this.init();
        this.loadScanList();
        this.setupEventListeners();
        this.animate();
    }

    init() {
        const container = document.getElementById('canvas-container');

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0f172a);
        this.scene.fog = new THREE.Fog(0x0f172a, 5, 60);

        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(this.renderer.domElement);

        const ambient = new THREE.AmbientLight(0xffffff, 0.7);
        this.scene.add(ambient);
        const directional = new THREE.DirectionalLight(0xffffff, 0.6);
        directional.position.set(5, 10, 5);
        this.scene.add(directional);

        this.controls = new THREE.PointerLockControls(this.camera, document.body);
        this.scene.add(this.controls.getObject());

        const overlay = document.getElementById('overlay');
        overlay.addEventListener('click', () => this.controls.lock());
        this.controls.addEventListener('lock', () => overlay.classList.add('hidden'));
        this.controls.addEventListener('unlock', () => overlay.classList.remove('hidden'));

        window.addEventListener('resize', () => this.onResize());
    }

    async loadScanList() {
        const select = document.getElementById('scan-select');
        try {
            const data = await fetch('api/scans').then(r => r.json());
            (data.scans || []).forEach(scan => {
                const option = document.createElement('option');
                option.value = scan.file;
                option.textContent = scan.name;
                select.appendChild(option);
            });

            const requested = new URLSearchParams(window.location.search).get('scan');
            const toLoad = requested && [...select.options].some(o => o.value === requested)
                ? requested
                : (data.scans && data.scans[0] ? data.scans[0].file : null);

            if (toLoad) {
                select.value = toLoad;
                this.loadScan(toLoad);
            }
        } catch (e) {
            console.error('Failed to load scan list', e);
        }
    }

    async loadScan(filename) {
        const response = await fetch(`models/${filename}`);
        const data = await response.json();

        if (this.object) {
            this.scene.remove(this.object);
            this.object.geometry.dispose();
            this.object.material.dispose();
        }

        const rawVertices = data.vertices;
        const n = rawVertices.length / 3;
        const positions = new Float32Array(rawVertices.length);

        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity; // remapped vertical (data.z)
        let minZ = Infinity, maxZ = -Infinity;

        for (let i = 0; i < n; i++) {
            const x = rawVertices[i * 3];
            const y = rawVertices[i * 3 + 1];
            const z = rawVertices[i * 3 + 2];

            // Remap: three.x = data.x, three.y = data.z (up), three.z = data.y
            positions[i * 3] = x;
            positions[i * 3 + 1] = z;
            positions[i * 3 + 2] = y;

            if (x < minX) minX = x; if (x > maxX) maxX = x;
            if (z < minY) minY = z; if (z > maxY) maxY = z;
            if (y < minZ) minZ = y; if (y > maxZ) maxZ = y;
        }

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        let mesh;
        if (data.faces && data.faces.length > 0) {
            geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(data.faces), 1));
            geometry.computeVertexNormals();
            const material = new THREE.MeshPhongMaterial({ color: 0x8892b0, side: THREE.DoubleSide, flatShading: false });
            mesh = new THREE.Mesh(geometry, material);
        } else {
            const material = new THREE.PointsMaterial({ color: 0x60a5fa, size: 0.04 });
            mesh = new THREE.Points(geometry, material);
        }

        this.object = mesh;
        this.scene.add(mesh);

        const margin = 1.0;
        this.bounds = {
            minX: minX - margin, maxX: maxX + margin,
            minY: minY + 0.3, maxY: maxY + margin * 2,
            minZ: minZ - margin, maxZ: maxZ + margin,
        };

        // Start roughly in the middle of the space, near the floor.
        const eyeHeight = 1.6;
        this.controls.getObject().position.set(
            (minX + maxX) / 2,
            Math.min(minY + eyeHeight, this.bounds.maxY),
            (minZ + maxZ) / 2,
        );
    }

    setupEventListeners() {
        document.getElementById('scan-select').addEventListener('change', (e) => {
            if (e.target.value) this.loadScan(e.target.value);
        });

        document.addEventListener('keydown', (e) => this.onKey(e, true));
        document.addEventListener('keyup', (e) => this.onKey(e, false));
    }

    onKey(e, pressed) {
        switch (e.code) {
            case 'KeyW': case 'ArrowUp': this.move.forward = pressed; break;
            case 'KeyS': case 'ArrowDown': this.move.back = pressed; break;
            case 'KeyA': case 'ArrowLeft': this.move.left = pressed; break;
            case 'KeyD': case 'ArrowRight': this.move.right = pressed; break;
            case 'Space': this.move.up = pressed; break;
            case 'ShiftLeft': case 'ShiftRight': this.move.down = pressed; break;
            case 'ControlLeft': case 'ControlRight': this.move.sprint = pressed; break;
        }
    }

    onResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }

    updateMovement(delta) {
        if (!this.controls.isLocked) return;

        const speed = this.baseSpeed * (this.move.sprint ? this.sprintMultiplier : 1);
        const damping = Math.max(0, 1 - delta * 6);
        this.velocity.x *= damping;
        this.velocity.z *= damping;

        const direction = new THREE.Vector3();
        direction.z = Number(this.move.forward) - Number(this.move.back);
        direction.x = Number(this.move.right) - Number(this.move.left);
        direction.normalize();

        if (this.move.forward || this.move.back) this.velocity.z -= direction.z * speed * delta * 10;
        if (this.move.left || this.move.right) this.velocity.x -= direction.x * speed * delta * 10;

        this.controls.moveRight(-this.velocity.x * delta);
        this.controls.moveForward(-this.velocity.z * delta);

        const obj = this.controls.getObject();
        if (this.move.up) obj.position.y += speed * delta;
        if (this.move.down) obj.position.y -= speed * delta;

        if (this.bounds) {
            obj.position.x = THREE.MathUtils.clamp(obj.position.x, this.bounds.minX, this.bounds.maxX);
            obj.position.y = THREE.MathUtils.clamp(obj.position.y, this.bounds.minY, this.bounds.maxY);
            obj.position.z = THREE.MathUtils.clamp(obj.position.z, this.bounds.minZ, this.bounds.maxZ);
        }
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        const delta = this.clock.getDelta();
        this.updateMovement(delta);
        this.renderer.render(this.scene, this.camera);
    }
}

window.addEventListener('DOMContentLoaded', () => {
    window.walkthrough = new WalkthroughViewer();
});
