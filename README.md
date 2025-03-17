
# Scanlite Analysis ROS Package

This package provides tools for real-time ultrasound bone segmentation, 3D reconstruction, and ROS integration, complete with intuitive graphical interfaces.

## Repository Structure
```
scanlite_analysis/
├── launch/
│   └── segmentation.launch
├── scripts/
│   ├── segmentation_node.py
│   ├── rosbagtest.py
│   ├── LiveDemoTool.py
│   ├── DataAcqTool.py
│   └── PyCATMAUS/
│       ├── SegBone.py
│       └── TransFunction.py
├── src/
│   └── PyCATMAUS/
│       ├── SegBone.py
│       └── TransFunction.py
└── package.xml

## ROS Nodes

### `segmentation_node.py`
- Publishes real-time segmentation data through ROS topics.

## GUI Tools

- **`DataAcqTool.py`**: GUI for acquiring live ultrasound data (Tkinter).
- **`LiveDemoTool.py`**: Real-time segmentation visualization (Tkinter).

## ROS Bag Testing Tool

- **`rosbagtest.py` 
:** Tool for selecting and playing ROS bag files to test the segmentation node.

## Functionalities

- **ROS Integration:**
  - Topics: `/us_image`, `/vicon/clarius_5_marker/clarius_5_marker`.
  - Motion synchronization: `bound_img_motion()`.

- **GUI Tools:**
  - Ultrasound image display with segmentation overlay.
  - Interactive control of segmentation parameters.

- **Bone Segmentation:**
  - Real-time segmentation with interactive parameter control.

- **3D Reconstruction:**
  - Converts 2D segmentation data to 3D using Vicon data.

- **User Interactivity:**
  - Interactive GUI buttons for real-time operations.

## Segmentation Parameters

| Parameter | Meaning                    | Effect on Segmentation                                      |
|-----------|----------------------------|-------------------------------------------------------------|
| **F0**    | Energy continuity weight   | Controls smoothness; higher values enforce smoother edges.  |
| **F1**    | Energy smoothness weight   | Penalizes abrupt depth changes; higher values smooth transitions. |
| **Bth**   | Bone threshold             | Pixel intensity threshold. Lower values include more pixels. |
| **JC**    | Jump constraint            | Limits allowed segmentation path jumps. Higher values increase flexibility. |

## System Diagram

```plaintext
ROS Topics (/us_image, Vicon motion)
  │
  ├── segmentation_node.py ───► Segmentation Results (ROS Topics)
  │
  ├── DataAcqTool.py (Live Data GUI)
  │
  ├── LiveDemoTool.py (Real-time Segmentation GUI)
  │
  └── rosbagtest.py (CatMausApp - ROS Bag Playback GUI)
        │
        └─ Interactive 3D Visualization
```

## Usage Examples

```bash
roslaunch scanlite_analysis segmentation.launch
```

