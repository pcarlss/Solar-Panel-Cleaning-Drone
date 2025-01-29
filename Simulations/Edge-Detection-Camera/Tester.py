import cv2
import numpy as np
import time
import tensorflow as tf
import tensorflow_hub as hub
import os

# Parameters for solar panel detection
EDGE_THRESHOLD_LOW = 20      # Further reduce the lower threshold for more refined edges
EDGE_THRESHOLD_HIGH = 60     # Lower the upper threshold to avoid detecting minor edges
MORPH_KERNEL = np.ones((5, 5), np.uint8)  # Smaller kernel for less aggressive morphological operation
MIN_LINE_LENGTH = 300       # Further increase the minimum line length to focus only on long lines (solar panels' edges)
MAX_LINE_GAP = 5           # Reduce max gap for more continuous line detection
GRID_SIZE = 10             # Edge matrix size
FRAME_RATE = 0.5           # Matrix output rate (1 matrix every 2 seconds)

MODEL_URL = "./Models"  # Make sure this points to the correct path
print("Loading DeepLabV3 model from TensorFlow Hub...")
# model = tf.saved_model.load(MODEL_URL)  # Load the model as a SavedModel
model = hub.load("https://www.kaggle.com/models/google/deeplab-edgetpu/TensorFlow2/default-argmax-m/1")

# Background subtractor
bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=True)


def adaptive_canny(image, sigma=0.33):
    """Apply Canny edge detection with dynamically adjusted thresholds."""
    median = np.median(image)
    lower = int(max(0, (1.0 - sigma) * median))
    upper = int(min(255, (1.0 + sigma) * median))
    edges = cv2.Canny(image, lower, upper)
    return edges


def preprocess_frame(frame):
    """Preprocess the frame with background subtraction and adaptive thresholding."""
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply background subtraction
    fg_mask = bg_subtractor.apply(frame)

    # Combine foreground mask with the grayscale frame
    combined = cv2.bitwise_and(gray, gray, mask=fg_mask)

    # Apply adaptive thresholding to enhance edges
    thresholded = cv2.adaptiveThreshold(combined, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    return thresholded


def semantic_segmentation(frame):
    """Perform semantic segmentation using a pre-trained DeepLabV3 model."""
    # Resize frame to the expected input size for the model (512x512)
    input_image = cv2.resize(frame, (512, 512))  # Resize to 512x512
    input_tensor = tf.convert_to_tensor(input_image, dtype=tf.float32)[tf.newaxis, ...] / 255.0

    # Run the segmentation model
    result = model(input_tensor)

    # The model likely returns a tensor with shape (batch_size, height, width, num_classes)
    # Apply tf.argmax to get the predicted class (most likely class 0-255 for segmentation)
    prediction = tf.argmax(result, axis=-1).numpy()[0]  # Select the class with the highest probability

    # Resize the segmentation map back to the original frame size
    segmented = cv2.resize(prediction.astype(np.uint8), (frame.shape[1], frame.shape[0]))
    return segmented


def detect_edges_and_lines(frame):
    """Detect prominent edges and filter out small debris or noise."""
    # Preprocess the frame
    preprocessed = preprocess_frame(frame)

    # Apply morphological operations with a smaller kernel to reduce noise
    edges = cv2.morphologyEx(preprocessed, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    # Detect lines using Probabilistic Hough Line Transform with stricter parameters
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=200,  # Increased threshold to only detect prominent lines
        minLineLength=MIN_LINE_LENGTH,  # Focus on longer lines
        maxLineGap=MAX_LINE_GAP        # Further reduce the max gap for line continuity
    )

    # Filter and refine detected lines
    filtered_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if line_length > MIN_LINE_LENGTH:  # Only consider longer lines
                filtered_lines.append((x1, y1, x2, y2))

    return edges, filtered_lines


def generate_edge_matrix(edges, grid_size):
    """Generate a binary matrix indicating the presence of edges within grid cells."""
    height, width = edges.shape
    region_height = height // grid_size
    region_width = width // grid_size

    # Initialize binary edge matrix
    edge_matrix = np.zeros((grid_size, grid_size), dtype=int)

    # Loop through the edges and assign them to the grid
    for y in range(grid_size):
        for x in range(grid_size):
            # Calculate the region of interest in the original image
            y_start, y_end = y * region_height, (y + 1) * region_height
            x_start, x_end = x * region_width, (x + 1) * region_width

            # Check if there are any edges in this region
            if np.any(edges[y_start:y_end, x_start:x_end]):
                edge_matrix[y, x] = 1

    return edge_matrix


def draw_lines_on_frame(frame, lines):
    """Draw detected lines on the frame for visualization."""
    for x1, y1, x2, y2 in lines:
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return frame


def main():
    """Main loop for edge detection and semantic segmentation."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to exit.")
    last_update_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Detect edges and lines
        edges, filtered_lines = detect_edges_and_lines(frame)

        # Perform semantic segmentation
        segmentation_map = semantic_segmentation(frame)

        # Generate edge matrix
        edge_matrix = generate_edge_matrix(edges, GRID_SIZE)

        # Draw lines on the frame
        frame_with_lines = draw_lines_on_frame(frame.copy(), filtered_lines)

        # Overlay the segmentation map on the frame
        segmentation_overlay = cv2.addWeighted(frame, 0.6, cv2.applyColorMap(segmentation_map * 15, cv2.COLORMAP_JET), 0.4, 0)

        # Display the results
        cv2.imshow("Edges", edges)
        cv2.imshow("Lines", frame_with_lines)
        cv2.imshow("Semantic Segmentation Overlay", segmentation_overlay)

        # Output edge matrix
        current_time = time.time()
        if current_time - last_update_time >= 1 / FRAME_RATE:
            print("Edge Matrix:")
            print(edge_matrix)
            last_update_time = current_time

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
