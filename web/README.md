# React Dashboard

Full documentation: https://example.github.io/cam_slicer/

This folder contains a simple dashboard component for the CAM Slicer project. It uses TailwindCSS for styling and displays a sidebar, main heading, button and recent project list. Clicking **New Project** is logged to `logs/central.log` using the small logger utility.

## Usage

```
import Dashboard from './Dashboard';

const projects = [
  { name: 'Panel Cut', date: '2025-07-01', type: 'GCODE' },
  { name: 'Logo Engrave', date: '2025-06-24', type: 'SVG' },
  { name: 'Bracket', date: '2025-06-20', type: 'DXF' },
];

<Dashboard projects={projects} /> // displays at most 3 entries
```

Include TailwindCSS via CDN or your build pipeline to style the component.

## File Uploader

`FileUploader` provides a drag-and-drop area for importing CAM files. It logs the chosen file and the continue action to `logs/central.log`, shows a simple preview, and displays a button to continue once a file is selected. Unsupported or oversized files show a clear error message. Supported formats are SVG, DXF, DWG, STL, OBJ, G-code and common image files.

```jsx
import FileUploader from './FileUploader';

<FileUploader onContinue={(file) => console.log(file.name)} />
```

## Toolpath Transform Controls

`ToolpathTransform` lets you rotate, scale and translate a toolpath interactively. Each change is logged to `logs/central.log`.

```jsx
import ToolpathTransform from './ToolpathTransform';

<ToolpathTransform onChange={(t) => console.log(t)} />
```

## G-code Streamer

`GcodeStreamer` streams a G-code array in blocks of five lines. It shows progress and the last streamed lines while logging actions to `logs/central.log`.

```jsx
import GcodeStreamer from './GcodeStreamer';

const lines = ["G1 X0 Y0", "G1 X1 Y0", "G1 X2 Y0", ...];

<GcodeStreamer gcodeLines={lines} fileName="demo.gcode" />
```

## Job Recovery Button

`ResumeJobButton` reads `last_block.json` and calls your callback when the user
chooses to resume a job.

```jsx
import ResumeJobButton from './ResumeJobButton';

<ResumeJobButton onResume={(info) => console.log('resume', info)} />
```

## Machine Monitor

`MachineMonitor` displays dummy machine status that updates every second.

```jsx
import MachineMonitor from './MachineMonitor';

<MachineMonitor />
```

## Webcam Detector

`WebcamDetector` shows a live webcam feed and simulates object detection. Click **Start Object Detection** to enable the camera. When a person or hand is detected a red warning banner appears and a message is logged to `logs/central.log`.

```jsx
import WebcamDetector from './WebcamDetector';

<WebcamDetector />
```

## Project Saver

`ProjectSaver` lets you save a project with optional notes and export G-code. Errors during export are shown as red text and all clicks are logged to `logs/central.log`.

```jsx
import ProjectSaver from './ProjectSaver';

const lines = ['G1 X0 Y0', 'G1 X1 Y0'];

<ProjectSaver gcodeLines={lines} />
```

`FeedbackForm` allows beta users to send notes that are appended to `logs/central.log`.

```jsx
import FeedbackForm from './FeedbackForm';

<FeedbackForm />
```


## Project Manager

`ProjectManager` saves and loads complete project data as JSON using FileSaver.js. Actions are logged to `logs/central.log` and invalid or oversized files show an error.

```jsx
import ProjectManager from './ProjectManager';

<ProjectManager toolpaths={paths} material="MDF" machineProfile={profile} settings={{feed:1000}} onLoad={data => console.log(data)} />
```
## Material Selector

`MaterialSelector` offers a dropdown for common materials and shows recommended spindle speed and feedrate. Selecting **Custom** displays an input field for your own material name. Every selection is logged to `logs/central.log`.

```jsx
import MaterialSelector from './MaterialSelector';

<MaterialSelector />
```

## Machine Profile Selector

`MachineProfileSelector` provides presets for GRBL and Smoothieware machines or a **Custom** profile. Values are stored in `localStorage` and each selection is logged.

```jsx
import MachineProfileSelector from './MachineProfileSelector';

<MachineProfileSelector />
```

## Multi-machine overview

The dashboard can manage several machines at once. Use `MachineMonitor` to show
status and assign jobs using the backend `MachineManager`.

```jsx
import MachineMonitor from './MachineMonitor';

<MachineMonitor name="router1" status={{state: 'idle'}} />
```

## Cutting Strategy Selector

`CuttingStrategySelector` lets the user pick a cutting approach. Each radio option shows a small icon and short description. The selected strategy is logged to `logs/central.log`.

```jsx
import CuttingStrategySelector from './CuttingStrategySelector';

<CuttingStrategySelector onChange={(s) => console.log(s)} />
```

## Layer Preview

`LayerPreview` shows sliced toolpath layers with a slider and play button.
```jsx
import LayerPreview from './LayerPreview';

const layers = [
  [[[0,0,0],[10,0,0]]],
  [[[0,0,0],[0,10,-1]]]
];

<LayerPreview layers={layers} />
```

## Tool List

`ToolList` manages a list of tools with name, diameter, type and maximum RPM. Tools are saved to `localStorage` so the list persists across reloads. Adding or removing a tool writes to `logs/central.log`.

```jsx
import ToolList from './ToolList';

<ToolList />
```

## Heightmap Viewer

`HeightmapViewer` loads a JSON heightmap and renders it as a 3D surface with Three.js. Colors range from green (low) to red (high) and hovering shows coordinates.

```jsx
import HeightmapViewer from './HeightmapViewer';

<HeightmapViewer />
```

## 3D Viewport

`Viewport3D` shows a CNC work area using **react-three-fiber** with orbit controls and a 400Ã400&nbsp;mm grid. Drag STL, OBJ, SVG, DXF or G-code onto the canvas to import a model. Imported items appear centered and listed with their dimensions and a Select button. When selected, on-canvas transform controls allow moving or rotating the model.

```tsx
import Viewport3D from './Viewport3D';

<Viewport3D />
```

When dropping a G-code file, its toolpaths appear in color on the model. Each
operation type can be shown or hidden, and the **Play** button animates the
toolhead along the visible path.

Use the slider to step through toolpath **layers** or **segments**. Pick the mode
from the dropdown beside the slider. Enable **Diagnostics** to show warning
markers for sharp corners or low Z moves. Enable **AI Diagnostics** to run a
basic analysis on imported models and toolpaths and display colored markers for
out-of-bound moves or deep cuts. Clicking a marker zooms the view to that
location and logs the action.

Use **Snapshot** to export the current viewport as PNG. **Export Report**
generates a PDF with the snapshot, model info and AI diagnostics, including
timestamp, project name and optional notes.

Click **Dist** or **Angle** to measure between picked points directly in the
viewport. Use **Note** to drop a yellow label anywhere on the scene. These
helpers log their actions to `logs/central.log`.

The viewport now shows multiple toolheads. Paths are coloured by head type and
icons mark each start point. Toggle heads in the sidebar and click **Optimize
Order** to reorder paths. *Material preview* tints the model based on the
operation.

Rendering is delegated to the modular `ToolpathRenderer` component so additional toolheads can be integrated.
The parser understands A/B/C rotations, allowing preview of 4â and 5âaxis programs with correct head orientation.
## Toolpath Simulator

`ToolpathSimulator` animates toolhead movement from a G-code string. Use the play, pause and step buttons to inspect each point. Actions are logged to `logs/central.log`.

The main `Viewport3D` component now supports full process simulation. Hit **Play** in the sidebar to watch material being removed in real time. Faded lines indicate remaining moves while completed sections are highlighted. You can pause or step through individual commands and progress is shown with a small bar.

```jsx
import ToolpathSimulator from './ToolpathSimulator';

const code = 'G1 X0 Y0\nG1 X10 Y0\nG1 X10 Y10';

<ToolpathSimulator gcode={code} />
```
## Cut Stock Simulator
`CutStockSimulator` runs a simple voxel-based removal simulation and lets you export the remaining stock to STL.

```jsx
import CutStockSimulator from "./CutStockSimulator";

<CutStockSimulator toolpaths={paths} stock={{x:50,y:50,z:10}} toolRadius={2} />
```

## VR / AR Viewer

`VRViewer` uses WebXR to preview toolpaths in a headset or AR-capable browser. A small sphere moves along the path and you can enable VR or AR mode.

```jsx
import VRViewer from './VRViewer';

const points = [ {x:0,y:0,z:0}, {x:20,y:0,z:0}, {x:20,y:20,z:-5} ];

<VRViewer points={points} mode="vr" />
```

Press **Play** to animate and use the displayed VR/AR button to enter immersive mode. Logging captures when the viewer starts and stops.


## Surface Analyzer

`SurfaceAnalyzer` compares a G-code string with a loaded heightmap and highlights collisions or areas where the tool loses contact. Results are logged to `logs/central.log`.

```jsx
import SurfaceAnalyzer from './SurfaceAnalyzer';

const gcode = 'G1 X0 Y0 Z0\nG1 X1 Y0 Z-1';
const map = { points: [ {x:0,y:0,z:0}, {x:1,y:0,z:0} ] };

<SurfaceAnalyzer gcode={gcode} heightmap={map} />
```

## Toolpath Optimizer

`ToolpathOptimizer` uses a dummy API call to adjust a toolpath based on material, depth per pass and risk. It logs the optimization and shows before/after previews.
The **Experimental features** panel exposes an *Adaptive Toolpath* checkbox which enables experimental surface-aware generation when checked.

```jsx
import ToolpathOptimizer from './ToolpathOptimizer';

const gcode = 'G1 X0 Y0\nG1 X1 Y0';
const heightmap = { points: [{x:0,y:0,z:0},{x:1,y:0,z:-0.1}] };

<ToolpathOptimizer gcode={gcode} heightmap={heightmap} />
```

## Setup Wizard

`SetupWizard` guides new users through importing a design, selecting material and tool, choosing a machine profile and finally exporting G-code. Navigation is done with **Next** and **Back** buttons while the progress bar at the top shows the current step. Actions are logged to `logs/central.log`.

```jsx
import SetupWizard from './SetupWizard';

const gcode = ['G1 X0 Y0', 'G1 X1 Y0'];

<SetupWizard gcodeLines={gcode} />
```

## Vector Editor

`VectorEditor` imports raster or vector files and provides node editing, path operations and a basic layer system. Raster images are traced using **OpenCV.js**. Set the threshold and choose outline or centerline extraction. You can duplicate or mirror paths, drag nodes, and every step is kept in an undo history and written to `logs/central.log`.

```jsx
import VectorEditor from './VectorEditor';

<VectorEditor />
```


## Haptic Toolpath Editor

`HapticToolpathEditor` lets you sketch paths freely with mouse or touch. Each stroke is placed on its own layer and you can undo/redo changes. Visible layers can be exported as G-code or previewed with the built in simulator. All actions are logged to `logs/central.log`.

```jsx
import HapticToolpathEditor from './HapticToolpathEditor';

## Operation Assigner

`OperationAssigner` lets you choose an operation (Cut, Engrave, Mill, Print) and assign a tool head (Laser, Spindle, Print Head) for each imported path. The `Tools & Operations` section of `CuraSlicerUI` embeds this component and also shows per-path parameters.

```jsx
import OperationAssigner from './OperationAssigner';

const paths = [[{x:0,y:0},{x:1,y:0}], [{x:0,y:1},{x:1,y:1}]];

<OperationAssigner paths={paths} />
```

## Cura Style UI

`CuraSlicerUI` provides a full layout inspired by Cura with a scrollable sidebar.
Each section expands to reveal options and logging occurs whenever you select a section or use the import/export buttons. The sidebar includes groups such as *Import & Model*, *Tools & Operations*, *Hybrid Operations*, *Toolpath Optimization*, *Surface & Sensors*, *Simulation*, *AI Analyze & Optimize*, *Export & Streaming*, *Monitoring*, *Experimental features* and *Diagnostics*. Each group shows a small emoji icon. The Toolpath Optimization section simply points to the AI panel for optimizers. The Simulation section lets you preview an animated toolpath and shows basic statistics.

The **AI Analyze & Optimize** panel lists loaded plugins with enable toggles and provides quick buttons for AI analysis or auto optimisation. The **Experimental features** panel hosts toggles like **Adaptive Toolpath**. The **Diagnostics** panel offers actions to run the project tests and generate a system report, logging everything to `logs/central.log`.
The new **Experimental AI** panel bundles the `SurfaceAnalyzer`, `ToolpathOptimizer` and a placeholder AI Feedback widget for advanced users.

The **Export & Streaming** section exports G-code for **GRBL**, **Marlin** or **Klipper**, optionally with a custom postprocessor, and can stream the code directly with a progress log or run a dry-run.

The **Monitoring** section displays dummy machine position, feed and state updating every second.

The **Tools & Operations** menu lets you assign operations and tool heads to paths, while **Hybrid Operations** previews those assignments overlayed on the canvas so you can verify the correct head is used for each path.
The **Experimental features** section holds toggles like *Adaptive Toolpath* and other workâinâprogress options.

Drag a supported file (SVG, DXF, DWG, OBJ, STL, JPG, PNG) onto the main canvas to import it. A preview appears centered on the canvas and the right panel lists file details with transform controls.

```jsx
import CuraSlicerUI from './CuraSlicerUI';

<CuraSlicerUI />
```

## Workspace scanning

`WorkspaceScanner` lets you choose **Camera only**, **Probe only** or
**Hybrid** scanning. The selected mode is logged and results appear as overlays
in `Viewport3D` when combined with your frontend logic.

```jsx
import WorkspaceScanner from './WorkspaceScanner';

<WorkspaceScanner onScan={(data) => console.log(data)} />
```

## Log Export

`LogExportButton` lets users download `logs/central.log` with one click so they can send it for support.
```jsx
import LogExportButton from "./LogExportButton";

<LogExportButton />
```

## Robot Arm Panel

`RobotArmPanel` manages robotic arm profiles and diagnostics. Enable robot mode,
run a **Dry run** for calibration and view real-time joint status. Profiles can
be edited, saved to `localStorage`, imported or exported. The panel warns about
overload, collisions or joint limit problems. Commands may be streamed directly
to the robot or exported from the panel.
The bottom of the panel includes a diagnostics dashboard and bug report form for beta feedback.

```jsx
import RobotArmPanel from './RobotArmPanel';

<RobotArmPanel />
```

### Robot Trajectory Optimizer
`RobotTrajectoryOptimizer` calls the backend `/robot_optimize` endpoint to smooth
robot arm paths and shows warnings for risky moves. Users can preview the
optimized path and accept or reject the suggestions.

### Base Position Planner
`plan_base_zones(toolpath, profile, [0,1000], [0,1000])` splits large jobs into
zones so the portal moves between optimal base positions automatically. Use
`optimize_base_position_ai` when debris or fixtures are detected via
`GeoFence` to select a safer base. The viewport highlights each zone and the
planned gantry coordinates.

### Workspace Zoning
`plan_workspace_zones` returns an ordered list of zones with move commands for
the gantry. Display this summary before running the job so the operator can
confirm each reposition step.

```jsx
import ZoneSummary from './ZoneSummary';

<ZoneSummary zones={zonePlan} />
```

To preview moves visually drop the zone JSON onto `Viewport3D` and enable the
zone controls. The **ZoneVisualizer** component animates base transitions and
lets you drag each cube to adjust the plan.

Detected debris or clamps can be previewed with **GeoFenceOverlay** which draws
red or blue rectangles over the workspace so the operator can confirm zones
before starting the job.

```jsx
import RobotTrajectoryOptimizer from './RobotTrajectoryOptimizer';

<RobotTrajectoryOptimizer toolpath={[[0,0,0],[10,0,0]]} profile={{}} />
```

## ROS2 Bridge

Connect the UI with a ROS2 controller via the backend `ROSBridge`. Status
updates appear in real time and control commands are forwarded to the machine.


### Mobile Performance
`ToolpathRenderer` uses reduced animation speed on small screens and is exported with `React.memo` to avoid unnecessary re-renders.
