import cv2
import numpy as np
import time

# Parameters
EDGE_THRESHOLD_LOW = 50
EDGE_THRESHOLD_HIGH = 150
MORPH_KERNEL = np.ones((5, 5), np.uint8)
MIN_LINE_LENGTH = 250  # Minimum pixel length of detected edges
MAX_LINE_GAP = 20  # Maximum gap between segments to be considered a single line
GRID_SIZE = 10  # Divide the frame into GRID_SIZE x GRID_SIZE grid
FRAME_RATE = 0.5  # Matrix output rate (1 matrix every 2 seconds)

# Initialize background subtractor for foreground detection
bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=350, varThreshold=100, detectShadows=True)

def adaptive_canny(image, sigma=0.33):
    """Apply Canny edge detection with dynamically adjusted thresholds."""
    median = np.median(image)
    lower = int(max(0, (1.0 - sigma) * median))
    upper = int(min(255, (1.0 + sigma) * median))
    edges = cv2.Canny(image, lower, upper)
    return edges

def detect_solar_panel_edges(frame):
    """
    Detect prominent edges of the solar panel and filter out small debris or noise.
    """
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply adaptive Canny edge detection
    edges = adaptive_canny(blurred)

    # Use morphological closing to fill small gaps in the edges
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, MORPH_KERNEL)

    # Detect lines using Probabilistic Hough Line Transform
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=100,  # Threshold to detect a line; lower values will detect more lines
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=MAX_LINE_GAP,
    )

    # Filter and refine detected lines
    filtered_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if length >= MIN_LINE_LENGTH:  # Only accept lines that are longer than the threshold
                filtered_lines.append((x1, y1, x2, y2))

    return edges, filtered_lines

def generate_edge_matrix(edges, grid_size):
    """
    Generate a binary matrix indicating the presence of edges within grid cells.
    """
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
            if np.any(edges[y_start:y_end, x_start:x_end] > EDGE_THRESHOLD_HIGH):
                edge_matrix[y, x] = 1

    return edge_matrix

def draw_lines_on_frame(frame, lines):
    """
    Draw detected lines on the frame for visualization.
    """
    for x1, y1, x2, y2 in lines:
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return frame

def main():
    """
    Main loop to capture webcam frames, detect edges, and output the filtered edge matrix.
    """
    # Open the webcam (default device index is 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to exit.")

    last_update_time = time.time()

    while True:
        # Capture a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            break

        # Apply background subtraction to get the foreground mask
        fg_mask = bg_subtractor.apply(frame)

        # Blur the background (i.e., mask the foreground)
        background_blurred = cv2.GaussianBlur(frame, (15, 15), 0)
        frame_with_bg_blurred = cv2.bitwise_and(background_blurred, background_blurred, mask=fg_mask)

        # Perform edge detection and line filtering on the foreground (only where mask is 1)
        edges, filtered_lines = detect_solar_panel_edges(frame_with_bg_blurred)

        # Generate edge matrix
        edge_matrix = generate_edge_matrix(edges, GRID_SIZE)

        # Draw detected lines on the frame
        frame_with_lines = draw_lines_on_frame(frame.copy(), filtered_lines)

        # Display the original frame with detected lines
        cv2.imshow("Detected Edges", edges)
        cv2.imshow("Filtered Lines", frame_with_lines)

        # Print the edge matrix at the desired frame rate
        current_time = time.time()
        if current_time - last_update_time >= 1 / FRAME_RATE:
            print("Edge Detection Matrix:")
            print(edge_matrix)
            last_update_time = current_time

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
