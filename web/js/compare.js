/**
 * Compare Scans
 * Loads two saved scans, calls /api/compare (backed by
 * src/data_processing/change_detection.py) and renders the baseline as a
 * translucent reference cloud with added points in green and removed
 * points in red.
 *
 * Uses the same axis remap as walkthrough.js (data.z -> three.y) purely
 * for consistent visual orientation across pages; the comparison math
 * itself runs server-side on the original coordinates.
 */
class CompareViewer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.group = new THREE.Group();

        this.elements = {
            baselineSelect: document.getElementById('baseline-select'),
            currentSelect: document.getElementById('current-select'),
            btnCompare: document.getElementById('btn-compare'),
            stats: document.getElementById('stats'),
            clusterList: document.getElementById('cluster-list'),
            loading: document.getElementById('loading'),
        };

        this.init();
        this.loadScanOptions();
        this.elements.btnCompare.addEventListener('click', () => this.runCompare());
        this.animate();
    }

    init() {
        const container = document.getElementById('canvas-container');

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0f172a);
        this.scene.add(this.group);

        this.camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
        this.camera.position.set(5, 4, 5);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(this.renderer.domElement);

        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;

        this.scene.add(new THREE.AmbientLight(0xffffff, 0.8));
        const grid = new THREE.GridHelper(20, 20, 0x334155, 0x1e293b);
        this.scene.add(grid);

        window.addEventListener('resize', () => this.onResize());
    }

    remap(flatArray) {
        const n = flatArray.length / 3;
        const out = new Float32Array(flatArray.length);
        for (let i = 0; i < n; i++) {
            out[i * 3] = flatArray[i * 3];
            out[i * 3 + 1] = flatArray[i * 3 + 2];
            out[i * 3 + 2] = flatArray[i * 3 + 1];
        }
        return out;
    }

    async loadScanOptions() {
        const data = await fetch('api/scans').then(r => r.json());
        const scans = data.scans || [];

        [this.elements.baselineSelect, this.elements.currentSelect].forEach(select => {
            scans.forEach(scan => {
                const option = document.createElement('option');
                option.value = scan.file;
                option.textContent = scan.name;
                select.appendChild(option);
            });
        });

        const params = new URLSearchParams(window.location.search);
        const baseline = params.get('baseline');
        const current = params.get('current');
        if (baseline) this.elements.baselineSelect.value = baseline;
        if (current) this.elements.currentSelect.value = current;
    }

    async runCompare() {
        const baseline = this.elements.baselineSelect.value;
        const current = this.elements.currentSelect.value;
        if (!baseline || !current) {
            alert('Select both a baseline and a current scan.');
            return;
        }

        this.elements.loading.classList.remove('hidden');
        try {
            const url = `api/compare?baseline=${encodeURIComponent(baseline)}&current=${encodeURIComponent(current)}`;
            const report = await fetch(url).then(r => r.json());
            if (report.error) {
                alert(`Comparison failed: ${report.error}`);
                return;
            }
            this.renderReport(report);
        } finally {
            this.elements.loading.classList.add('hidden');
        }
    }

    renderReport(report) {
        while (this.group.children.length > 0) {
            const obj = this.group.children.pop();
            obj.geometry.dispose();
            obj.material.dispose();
        }

        const addedFlat = new Float32Array(report.added_points.flat());
        const removedFlat = new Float32Array(report.removed_points.flat());

        if (addedFlat.length > 0) {
            const geom = new THREE.BufferGeometry();
            geom.setAttribute('position', new THREE.BufferAttribute(this.remap(addedFlat), 3));
            const mat = new THREE.PointsMaterial({ color: 0x22c55e, size: 0.06 });
            this.group.add(new THREE.Points(geom, mat));
        }

        if (removedFlat.length > 0) {
            const geom = new THREE.BufferGeometry();
            geom.setAttribute('position', new THREE.BufferAttribute(this.remap(removedFlat), 3));
            const mat = new THREE.PointsMaterial({ color: 0xef4444, size: 0.06 });
            this.group.add(new THREE.Points(geom, mat));
        }

        this.frameCamera();

        this.elements.stats.innerHTML = `
            <div class="stat-row"><span>Alignment fitness</span><span class="v">${report.fitness.toFixed(2)}</span></div>
            <div class="stat-row"><span>Added points</span><span class="v">${report.added_count.toLocaleString()}</span></div>
            <div class="stat-row"><span>Removed points</span><span class="v">${report.removed_count.toLocaleString()}</span></div>
            <div class="stat-row"><span>Unchanged points</span><span class="v">${report.persistent_count.toLocaleString()}</span></div>
        `;

        this.elements.clusterList.innerHTML = report.clusters.map((cluster, i) => `
            <div class="cluster-item ${cluster.change_type}" data-index="${i}">
                ${cluster.change_type === 'added' ? '+ Added' : '− Removed'} region
                (${cluster.num_points} pts) at
                [${cluster.centroid.map(v => v.toFixed(1)).join(', ')}]
            </div>
        `).join('') || '<p style="color:#94a3b8;font-size:13px;">No discrete change clusters found.</p>';

        this.elements.clusterList.querySelectorAll('.cluster-item').forEach(el => {
            el.addEventListener('click', () => {
                const cluster = report.clusters[parseInt(el.dataset.index, 10)];
                const [x, y, z] = cluster.centroid; // data axes
                this.controls.target.set(x, z, y); // remapped to three.js axes
                this.camera.position.set(x + 2, z + 2, y + 2);
                this.controls.update();
            });
        });
    }

    frameCamera() {
        const box = new THREE.Box3().setFromObject(this.group);
        if (box.isEmpty()) return;
        const center = new THREE.Vector3();
        const size = new THREE.Vector3();
        box.getCenter(center);
        box.getSize(size);
        const maxDim = Math.max(size.x, size.y, size.z, 1);
        this.camera.position.set(center.x + maxDim, center.y + maxDim * 0.7, center.z + maxDim);
        this.controls.target.copy(center);
        this.controls.update();
    }

    onResize() {
        const container = document.getElementById('canvas-container');
        this.camera.aspect = container.clientWidth / container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(container.clientWidth, container.clientHeight);
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

window.addEventListener('DOMContentLoaded', () => {
    window.compareViewer = new CompareViewer();
});
