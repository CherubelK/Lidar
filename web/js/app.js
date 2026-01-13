/**
 * Application Controller
 * Manages UI interactions and viewer integration
 */

let viewer;
let availableModels = [];

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    console.log('Initializing 3D Trail Viewer...');

    // Create viewer
    viewer = new TrailViewer('canvas3d');

    // Load available models
    loadAvailableModels();

    // Set up event listeners
    setupEventListeners();

    // Start stats update
    setInterval(updateStatsDisplay, 100);

    console.log('Application initialized');
});

/**
 * Load list of available trail models
 */
function loadAvailableModels() {
    // In a real application, this would fetch from a server
    // For now, we'll check for models in the models directory

    // Example models list
    availableModels = [
        { name: 'Test Trail', file: 'models/test_trail.json', date: '2026-01-12' },
        // Add more models as they become available
    ];

    populateModelSelect();
}

/**
 * Populate the model selection dropdown
 */
function populateModelSelect() {
    const select = document.getElementById('model-select');

    availableModels.forEach((model, index) => {
        const option = document.createElement('option');
        option.value = index;
        option.textContent = model.name;
        select.appendChild(option);
    });

    // Load first model if available
    if (availableModels.length > 0) {
        loadModel(0);
    }
}

/**
 * Load a trail model
 */
function loadModel(index) {
    const model = availableModels[index];

    if (!model) {
        console.error('Model not found:', index);
        return;
    }

    console.log('Loading model:', model.name);

    // Show loading indicator
    showLoading(true);

    // Update trail info
    document.getElementById('trail-name').textContent = model.name;
    document.getElementById('scan-date').textContent = model.date || '-';

    // Load the mesh
    viewer.loadMesh(model.file)
        .then(() => {
            console.log('Model loaded successfully');
            showLoading(false);

            // Update stats
            const stats = viewer.getStats();
            document.getElementById('total-points').textContent = stats.vertices.toLocaleString();
            document.getElementById('mesh-faces').textContent = stats.faces.toLocaleString();
        })
        .catch((error) => {
            console.error('Failed to load model:', error);
            showLoading(false);
            alert(`Failed to load trail model: ${error.message}\n\nMake sure the model file exists in the models/ directory.`);
        });
}

/**
 * Show/hide loading indicator
 */
function showLoading(show) {
    const loading = document.getElementById('loading');
    if (show) {
        loading.classList.remove('hidden');
    } else {
        loading.classList.add('hidden');
    }
}

/**
 * Set up UI event listeners
 */
function setupEventListeners() {
    // Model selection
    document.getElementById('model-select').addEventListener('change', (e) => {
        const index = parseInt(e.target.value);
        if (!isNaN(index)) {
            loadModel(index);
        }
    });

    // Display mode
    document.getElementById('display-mode').addEventListener('change', (e) => {
        viewer.setDisplayMode(e.target.value);
    });

    // Color mode
    document.getElementById('color-mode').addEventListener('change', (e) => {
        viewer.setColorMode(e.target.value);
    });

    // Grid toggle
    document.getElementById('show-grid').addEventListener('change', (e) => {
        viewer.toggleGrid(e.target.checked);
    });

    // Axes toggle
    document.getElementById('show-axes').addEventListener('change', (e) => {
        viewer.toggleAxes(e.target.checked);
    });

    // Auto rotate
    document.getElementById('auto-rotate').addEventListener('change', (e) => {
        viewer.setAutoRotate(e.target.checked);
    });
}

/**
 * Update stats display
 */
function updateStatsDisplay() {
    const stats = viewer.getStats();

    document.getElementById('fps').textContent = stats.fps;
    document.getElementById('cam-pos').textContent =
        `(${stats.cameraPosition.x}, ${stats.cameraPosition.y}, ${stats.cameraPosition.z})`;
}

/**
 * Camera view presets
 */
function setCameraView(view) {
    viewer.setCameraView(view);
}

/**
 * Take screenshot
 */
function takeScreenshot() {
    viewer.takeScreenshot();
}

/**
 * Export data
 */
function exportData() {
    alert('Export functionality coming soon!\n\nYou can already find exported models in:\n- data/processed/ (NPZ format)\n- web/models/ (JSON, OBJ, PLY formats)');
}

/**
 * Load a custom trail from file
 */
function loadCustomTrail() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';

    input.onchange = (e) => {
        const file = e.target.files[0];
        if (file) {
            const reader = new FileReader();

            reader.onload = (event) => {
                try {
                    const meshData = JSON.parse(event.target.result);
                    viewer.createMeshFromData(meshData);
                    document.getElementById('trail-name').textContent = file.name;
                } catch (error) {
                    console.error('Error loading custom trail:', error);
                    alert('Failed to load trail file. Please ensure it is a valid JSON mesh file.');
                }
            };

            reader.readAsText(file);
        }
    };

    input.click();
}