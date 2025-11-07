---
title: CNC Batik Sketch Generator
emoji: 🎨
colorFrom: blue
colorTo: purple
sdk: gradio
sdk_version: 4.44.0
app_file: app_gradio.py
pinned: false
license: mit
---

# 🎨 CNC Batik Sketch Generator

Convert your images to G-code for CNC drawing machines!

## Features

- 📸 Upload any image (JPG, PNG, etc.)
- 🔧 Adjustable edge detection parameters
- 📐 Auto-scaled to A4 size (297mm x 210mm)
- 📥 Download G-code file ready for CNC machines
- ⚡ Real-time edge detection preview

## How to Use

1. **Upload an image** - Click on the upload area and select your image
2. **Adjust parameters** (optional) - Fine-tune the edge detection settings
3. **Generate G-code** - Click the "Generate G-code" button
4. **Download** - Download the generated G-code file
5. **Use with CNC** - Load the G-code into your CNC machine

## Parameters Explained

- **Blur Kernel**: Smoothing level before edge detection (higher = smoother)
- **Canny Low/High Threshold**: Edge detection sensitivity
- **Feed Rate**: Speed of CNC movement (mm/min)
- **Pen Up/Down Position**: Z-axis positions for pen control

## Technology Stack

- **OpenCV**: Image processing and edge detection
- **NumPy**: Numerical computations
- **Gradio**: User interface
- **Python**: Backend processing

## Output Format

The generated G-code is compatible with standard CNC machines and includes:
- G21 (metric units)
- G90 (absolute positioning)
- Optimized contour paths
- Pen up/down commands

## Perfect For

- Batik pattern creation
- Sketch drawing
- Line art reproduction
- CNC plotter projects

---

Made with ❤️ for CNC enthusiasts
