import gradio as gr
import cv2
import numpy as np
from PIL import Image
import io

def process_image(image, blur_kernel=5, canny_low=50, canny_high=150):
    """Convert image to edge detection"""
    img_array = np.array(image)
    
    # Convert to grayscale
    if len(img_array.shape) == 3:
        if img_array.shape[2] == 4:
            img_array = cv2.cvtColor(img_array, cv2.COLOR_RGBA2RGB)
        gray = cv2.cvtColor(img_array, cv2.COLOR_RGB2GRAY)
    else:
        gray = img_array
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)
    
    # Detect edges
    edges = cv2.Canny(blurred, canny_low, canny_high)
    
    return edges


def generate_gcode(edge_image, feed_rate=1000, pen_up_pos=5, pen_down_pos=0):
    """Generate G-code from edge image, scaled to A4 size"""
    height, width = edge_image.shape
    
    # A4 dimensions in mm (landscape)
    max_width_mm = 297.0
    max_height_mm = 210.0
    
    # Calculate scale factor to fit A4
    scale_x = max_width_mm / width
    scale_y = max_height_mm / height
    scale_factor = min(scale_x, scale_y)
    
    final_width_mm = width * scale_factor
    final_height_mm = height * scale_factor
    
    # Find contours
    contours, _ = cv2.findContours(edge_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Generate G-code
    gcode = []
    
    # Header
    gcode.append("G21")  # Set units to mm
    gcode.append("G90")  # Absolute positioning
    gcode.append("G92 X0 Y0 Z0")  # Set current position as zero
    gcode.append(f"G0 Z{pen_up_pos} F{feed_rate}")  # Lift pen
    gcode.append(f"G0 X0 Y0 F{feed_rate}")  # Move to origin with feed rate
    
    # Draw each contour
    for contour in contours:
        if len(contour) < 5:  # Skip small contours
            continue
  
        epsilon = 0.5  # nilai 0.5 mm cocok untuk skala A4
        contour = cv2.approxPolyDP(contour, epsilon, True)
        
        # Move to start of contour
        start_x = contour[0][0][0] * scale_factor
        start_y = (height - contour[0][0][1]) * scale_factor  # Flip Y axis
        
        gcode.append(f"G0 X{start_x:.3f} Y{start_y:.3f} F{feed_rate}")  # Move to start with feed rate
        gcode.append(f"G0 Z{pen_down_pos} F{feed_rate}")  # Lower pen
        
        # Draw contour
        for point in contour[1:]:
            x = point[0][0] * scale_factor
            y = (height - point[0][1]) * scale_factor  # Flip Y axis
            gcode.append(f"G1 X{x:.3f} Y{y:.3f} F{feed_rate}")
        
        # Lift pen
        gcode.append(f"G0 Z{pen_up_pos} F{feed_rate}")  # Lift pen with feed rate
    
    # Footer
    gcode.append(f"G0 X0 Y0 F{feed_rate}")  # Return to origin with feed rate
    gcode.append("M5")  # Stop spindle/tool
    gcode.append("M30")  # End program
    
    return gcode, final_width_mm, final_height_mm


def process_and_generate(image, blur, canny_low, canny_high, feed_rate, pen_up, pen_down):
    """Main processing function for Gradio"""
    if image is None:
        return None, "Please upload an image first!", None
    
    try:
        # Ensure blur kernel is odd
        if blur % 2 == 0:
            blur += 1
        
        # Process image
        edges = process_image(image, blur, canny_low, canny_high)
        
        # Generate G-code
        gcode_lines, width, height = generate_gcode(
            edges, 
            feed_rate, 
            pen_up, 
            pen_down
        )
        
        # Convert edges to PIL Image for display
        edges_pil = Image.fromarray(edges)
        
        # Create G-code text
        gcode_text = "\n".join(gcode_lines)
        
        # Create info text
        info = f"""
✅ Processing Complete!

📐 Output Dimensions:
- Width: {width:.2f} mm
- Height: {height:.2f} mm
- Total G-code lines: {len(gcode_lines)}

📄 Download the G-code file below and use it with your CNC machine.
        """
        
        # Save G-code to file
        gcode_file = "output.gcode"
        with open(gcode_file, 'w') as f:
            f.write(gcode_text)
        
        return edges_pil, info, gcode_file
    
    except Exception as e:
        return None, f"❌ Error: {str(e)}", None


# Create Gradio interface
with gr.Blocks(title="CNC Batik Sketch Generator", theme=gr.themes.Soft()) as demo:
    gr.Markdown("""
    # 🎨 CNC Batik Sketch Generator
    
    Convert your images to G-code for CNC drawing machines!
    
    **How to use:**
    1. Upload an image
    2. Adjust parameters (optional)
    3. Click "Generate G-code"
    4. Download the G-code file
    """)
    
    with gr.Row():
        with gr.Column():
            gr.Markdown("### 📤 Input")
            input_image = gr.Image(type="pil", label="Upload Image")
            
            with gr.Accordion("⚙️ Advanced Settings", open=False):
                blur = gr.Slider(1, 15, value=5, step=2, label="Blur Kernel (odd numbers only)")
                canny_low = gr.Slider(0, 200, value=50, step=10, label="Canny Low Threshold")
                canny_high = gr.Slider(0, 300, value=150, step=10, label="Canny High Threshold")
                feed_rate = gr.Slider(100, 3000, value=1000, step=100, label="Feed Rate (mm/min)")
                pen_up = gr.Slider(0, 10, value=5, step=0.5, label="Pen Up Position (mm)")
                pen_down = gr.Slider(0, 5, value=0, step=0.5, label="Pen Down Position (mm)")
            
            process_btn = gr.Button("🚀 Generate G-code", variant="primary", size="lg")
        
        with gr.Column():
            gr.Markdown("### 📊 Output")
            output_image = gr.Image(type="pil", label="Edge Detection Preview")
            info_text = gr.Textbox(label="Processing Info", lines=10)
            gcode_file = gr.File(label="📥 Download G-code")
    
    # Examples
    gr.Markdown("### 💡 Example Settings")
    gr.Examples(
        examples=[
            [5, 50, 150, 1000, 5, 0],
            [7, 30, 100, 1500, 5, 0],
            [3, 70, 200, 800, 5, 0],
        ],
        inputs=[blur, canny_low, canny_high, feed_rate, pen_up, pen_down],
        label="Try these presets"
    )
    
    # Connect button to processing function
    process_btn.click(
        fn=process_and_generate,
        inputs=[input_image, blur, canny_low, canny_high, feed_rate, pen_up, pen_down],
        outputs=[output_image, info_text, gcode_file]
    )
    
    gr.Markdown("""
    ---
    ### 📝 About
    This app converts images to G-code for CNC drawing machines using edge detection.
    Perfect for creating batik patterns and sketches!
    
    **Technology:** OpenCV, NumPy, Gradio
    """)

# Launch the app
if __name__ == "__main__":
    demo.launch()
