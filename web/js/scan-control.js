/**
 * Scan Control page logic: start/stop a scan session, poll live status,
 * and list previously saved scans for viewing/walkthrough/comparison.
 */
class ScanControl {
    constructor() {
        this.pollHandle = null;
        this.elements = {
            sessionName: document.getElementById('session-name'),
            duration: document.getElementById('duration'),
            btnStart: document.getElementById('btn-start'),
            btnStop: document.getElementById('btn-stop'),
            phaseBadge: document.getElementById('phase-badge'),
            statElapsed: document.getElementById('stat-elapsed'),
            statPoints: document.getElementById('stat-points'),
            statPackets: document.getElementById('stat-packets'),
            errorMsg: document.getElementById('error-msg'),
            library: document.getElementById('scan-library'),
        };

        this.elements.btnStart.addEventListener('click', () => this.startScan());
        this.elements.btnStop.addEventListener('click', () => this.stopScan());

        this.refreshStatus();
        this.loadLibrary();
        this.pollHandle = setInterval(() => this.refreshStatus(), 1000);
    }

    async startScan() {
        const name = this.elements.sessionName.value.trim() || `scan_${Date.now()}`;
        const duration = this.elements.duration.value.trim();

        let url = `api/scan/start?name=${encodeURIComponent(name)}`;
        if (duration) url += `&duration=${encodeURIComponent(duration)}`;

        const result = await fetch(url).then(r => r.json());
        if (!result.ok) {
            this.elements.errorMsg.textContent = result.error || 'Failed to start scan';
        } else {
            this.elements.errorMsg.textContent = '';
        }
        this.refreshStatus();
    }

    async stopScan() {
        const result = await fetch('api/scan/stop').then(r => r.json());
        if (!result.ok) {
            this.elements.errorMsg.textContent = result.error || 'Failed to stop scan';
        }
        this.refreshStatus();
    }

    async refreshStatus() {
        let status;
        try {
            status = await fetch('api/scan/status').then(r => r.json());
        } catch (e) {
            return;
        }

        const phase = status.phase || (status.running ? 'scanning' : 'idle');
        const badge = this.elements.phaseBadge;
        badge.textContent = phase;
        badge.className = `phase-badge phase-${phase}`;

        this.elements.statElapsed.textContent = status.elapsed_seconds != null ? `${status.elapsed_seconds}s` : '-';
        this.elements.statPoints.textContent = status.points_captured != null ? status.points_captured.toLocaleString() : '-';
        this.elements.statPackets.textContent = status.packet_count != null ? status.packet_count.toLocaleString() : '-';

        this.elements.errorMsg.textContent = status.error || '';

        const running = !!status.running;
        this.elements.btnStart.disabled = running;
        this.elements.btnStop.disabled = !running;

        if (phase === 'completed') {
            this.loadLibrary();
        }
    }

    async loadLibrary() {
        let data;
        try {
            data = await fetch('api/scans').then(r => r.json());
        } catch (e) {
            this.elements.library.innerHTML = '<div class="empty-state">Could not load scan library.</div>';
            return;
        }

        const scans = data.scans || [];
        if (scans.length === 0) {
            this.elements.library.innerHTML = '<div class="empty-state">No saved scans yet. Start one above.</div>';
            return;
        }

        const rows = scans.map(scan => {
            const sizeKb = scan.size_bytes ? (scan.size_bytes / 1024).toFixed(0) + ' KB' : '-';
            const file = encodeURIComponent(scan.file);
            return `
                <tr>
                    <td>${scan.name}</td>
                    <td>${sizeKb}</td>
                    <td>
                        <a href="viewer.html?scan=${file}">View</a>
                        <a href="walkthrough.html?scan=${file}">Walkthrough</a>
                        <a href="compare.html?baseline=${file}">Compare as baseline</a>
                    </td>
                </tr>`;
        }).join('');

        this.elements.library.innerHTML = `
            <table>
                <thead><tr><th>Name</th><th>Size</th><th>Actions</th></tr></thead>
                <tbody>${rows}</tbody>
            </table>`;
    }
}

window.addEventListener('DOMContentLoaded', () => {
    window.scanControl = new ScanControl();
});
