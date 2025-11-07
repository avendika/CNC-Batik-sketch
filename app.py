# app.py - Image Processing Only (No MQTT)
from flask import Flask, render_template, request, jsonify
import cv2
import numpy as np
from PIL import Image
import base64
import io
import os

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = 'static/uploads'
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024  # 16MB max

os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

# ===== IMAGE PROCESSING FUNCTIONS =====

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


def image_to_base64(image_array):
    """Convert numpy array to base64 string"""
    img = Image.fromarray(image_array)
    buffer = io.BytesIO()
    img.save(buffer, format='PNG')
    img_str = base64.b64encode(buffer.getvalue()).decode()
    return img_str


# ===== ROUTES =====

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/process_image', methods=['POST'])
def process_image_route():
    """Process uploaded image and return G-code"""
    try:
        # Get uploaded file
        if 'image' not in request.files:
            return jsonify({'error': 'No image uploaded'}), 400
        
        file = request.files['image']
        if file.filename == '':
            return jsonify({'error': 'Empty filename'}), 400
        
        # Get parameters
        blur = int(request.form.get('blur', 5))
        canny_low = int(request.form.get('canny_low', 50))
        canny_high = int(request.form.get('canny_high', 150))
        feed_rate = int(request.form.get('feed_rate', 1000))
        pen_up = float(request.form.get('pen_up', 5))
        pen_down = float(request.form.get('pen_down', 0))
        
        # Ensure blur kernel is odd
        if blur % 2 == 0:
            blur += 1
        
        # Load image
        image = Image.open(file.stream)
        
        # Process image
        edges = process_image(image, blur, canny_low, canny_high)
        
        # Generate G-code
        gcode_lines, width, height = generate_gcode(
            edges, 
            feed_rate, 
            pen_up, 
            pen_down
        )
        
        # Convert edges to base64 for preview
        edges_base64 = image_to_base64(edges)
        
        return jsonify({
            'success': True,
            'gcode': gcode_lines,
            'edges_base64': edges_base64,
            'width': width,
            'height': height,
            'total_lines': len(gcode_lines)
        })
    
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({'status': 'ok', 'service': 'image_processing'})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)