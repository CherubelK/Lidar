# 3D Trail Viewer

Interactive web-based 3D visualization for LiDAR-scanned hiking trails.

## Features

- **Interactive 3D Visualization** - Rotate, pan, and zoom to explore trails
- **Multiple Display Modes** - Solid, wireframe, and point cloud views
- **Color Coding** - Height-based elevation coloring
- **Camera Presets** - Quick views from different angles
- **Screenshot Export** - Save snapshots of your trails
- **Responsive Design** - Works on desktop and tablet

## Quick Start

### Option 1: Using Python HTTP Server

```bash
# From the web directory
python server.py
```

This will:
- Start a local web server on port 8000
- Automatically open your browser
- Serve the 3D viewer application

### Option 2: Using Python's Built-in Server

```bash
# From the web directory
python -m http.server 8000
```

Then open: `http://localhost:8000/index.html`

### Option 3: Direct File Access

Some browsers allow opening `index.html` directly, but this may have limitations due to CORS policies. Use one of the server options above for best results.

## Usage

1. **Load a Trail**
   - Select a trail from the dropdown menu
   - The 3D model will load automatically

2. **Navigate**
   - **Rotate**: Left click + drag
   - **Pan**: Right click + drag
   - **Zoom**: Scroll wheel
   - **Reset**: Double click

3. **Customize View**
   - Change display mode (Solid/Wireframe/Points)
   - Toggle grid and axes
   - Enable auto-rotation
   - Switch color modes

4. **Camera Presets**
   - Top View - Overhead view
   - Side View - Profile view
   - Front View - Head-on view
   - Perspective - Default 3D view

5. **Export**
   - Take screenshots with the camera button
   - Export data in various formats

## File Structure

```
web/
├── index.html          # Main HTML file
├── css/
│   └── style.css      # Styling
├── js/
│   ├── viewer.js      # 3D viewer engine
│   └── app.js         # Application logic
├── models/            # Trail 3D models (JSON, OBJ, PLY)
│   └── test_trail.*
├── server.py          # Development server
└── README.md          # This file
```

## Supported Formats

The viewer loads models in **JSON format** (Three.js BufferGeometry).

Other formats are also exported for use in 3D software:
- **OBJ** - Wavefront format (universal)
- **PLY** - Polygon File Format (good for point clouds)
- **JSON** - Three.js native format (for web)

## Adding New Trails

To add a new trail to the viewer:

1. **Generate the trail data** using the scanning pipeline:
   ```bash
   python examples/scan_and_build_map.py your_trail_name --duration 60
   ```

2. **Files will be automatically exported** to `web/models/`

3. **Update the model list** in `js/app.js`:
   ```javascript
   availableModels = [
       { name: 'Test Trail', file: 'models/test_trail.json', date: '2026-01-12' },
       { name: 'Your Trail', file: 'models/your_trail_name.json', date: '2026-01-13' },
   ];
   ```

4. **Refresh the page** - your trail will appear in the dropdown

## Customization

### Change Colors

Edit the color scheme in `js/viewer.js`:

```javascript
const colorLow = new THREE.Color(0x4a148c);   // Purple (low elevation)
const colorMid = new THREE.Color(0x2e7d32);   // Green (mid elevation)
const colorHigh = new THREE.Color(0xf57f17);  // Orange (high elevation)
```

### Adjust Camera

Modify camera settings in `js/viewer.js`:

```javascript
this.camera = new THREE.PerspectiveCamera(
    75,      // Field of view
    aspect,  // Aspect ratio
    0.1,     // Near clipping plane
    1000     // Far clipping plane
);
```

### Change Background

Update scene background in `js/viewer.js`:

```javascript
this.scene.background = new THREE.Color(0x0a0a0a);
```

## Browser Compatibility

Tested and working on:
- Chrome/Edge (Recommended)
- Firefox
- Safari
- Opera

Requires WebGL support (available in all modern browsers).

## Performance Tips

For large trail models:
1. Use downsampling in the processing pipeline
2. Enable wireframe mode for smoother navigation
3. Disable shadows in `viewer.js` if needed
4. Consider mesh decimation for very large datasets

## Troubleshooting

**Model not loading:**
- Check browser console for errors (F12)
- Verify JSON file exists in `models/` directory
- Ensure web server is running (don't use direct file access)
- Check file path in `app.js`

**Slow performance:**
- Try wireframe mode
- Reduce point cloud density
- Close other browser tabs
- Use a more powerful GPU

**Controls not working:**
- Make sure OrbitControls.js is loaded
- Check for JavaScript errors in console
- Try refreshing the page

## Technologies Used

- **Three.js** - 3D rendering engine
- **OrbitControls** - Camera navigation
- **Vanilla JavaScript** - No frameworks needed
- **HTML5/CSS3** - Modern web standards

## License

Part of the Unitree L2 LiDAR Trail Mapping Project.

## Credits

Built with [Three.js](https://threejs.org/) - JavaScript 3D library.