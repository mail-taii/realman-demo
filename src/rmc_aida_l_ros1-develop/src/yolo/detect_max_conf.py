import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.models.sam import Predictor as SAMPredictor

def choose_model():
    model_weight = '/home/rm/yolo/sam_b.pt'
    predictor = SAMPredictor(overrides=dict(conf=0.25, model=model_weight, save=False))
    return predictor

def detect_objects_with_clip(image, target_class, use_clip=False):
    """
    Detect objects in the image using YOLOv8. If use_clip is True, it will perform additional filtering based on CLIP model.
    """
    model = YOLO("/home/rm/yolo/best1.pt")  # Using YOLO-World model
    results = model.predict(image)
    
    # Get detection results: boxes and labels
    boxes = results[0].boxes  # The bounding boxes
    labels = results[0].names  # The class names for each box
    #print("Detected labels:", labels)

    # Get the detected classes based on boxes and confidences
    detected_classes = []
    valid_boxes = []
    for idx, box in enumerate(boxes):  # Use boxes.conf to get the confidence score
        if box.conf > 0.1:  # Filter out boxes with low confidence (e.g., < 0.25)
            if (not use_clip) or (use_clip and labels[box.cls[0].item()] == target_class):
                detected_classes.append(labels[box.cls[0].item()])  # Get class name from index
                valid_boxes.append(box)

    print(f"Detected classes: {detected_classes}")
    max_conf_box = None
    max_conf_class = None
    if detected_classes:
        max_conf_box = sorted(valid_boxes, key=lambda x: x.conf, reverse=True)[0]
        max_conf_class = labels[max_conf_box.cls[0].item()]

    return max_conf_box, max_conf_class


def process_results(results):
    """
    This function processes the results of SAM segmentation, returning the center and mask.
    """
    center = (0, 0)
    mask = None

    if results[0].masks is not None:
        for index, contour in enumerate(results[0].masks.xy):
            contour = contour.astype(np.int32)
            rect = cv2.minAreaRect(contour)
            center, wh, angle = rect
            center = list(map(int, center))

        masks = results[0].masks.data.clone()
        if isinstance(masks[0], torch.Tensor):
            masks = np.array(masks.cpu())
        
        mask = np.zeros_like(masks[0], dtype=np.uint8)
        for i in range(len(masks)):
            mask = cv2.morphologyEx(masks[i].astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) #   形态学操作，消除小孔
            mask[mask == 1] = 255

    return center, mask

def segment_image(color_img, target_class="apple", use_clip=False, output_path='segmentation_result.png', draw_path='detection_result.png'):
    """
    Main function for processing image. It will either detect objects with a specific class or detect all objects.
    """
    target_class = "banana"
    use_clip = False

    # Detect objects in the image
    box, label = detect_objects_with_clip(color_img, target_class, use_clip)

    if box:
        # Show the image with bounding boxes
        x1, y1, x2, y2 = box.xyxy[0]
        x_center = (x1 + x2) / 2  
        y_center = (y1 + y2) / 2  
        center = (x_center, y_center) 
        #print(f"Drawing box for class: {label_name}")
        cv2.rectangle(color_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        confidence = box.conf.item()
        conf_text = f"{confidence:.2f}" 
        display_text = f"{label}: {conf_text}" 
        cv2.putText(
            color_img,
            display_text,            
            (int(x1), int(y1) - 10), 
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),             
            2
        )
        
        # Save the image with bounding boxes
        cv2.imwrite(draw_path, color_img)

        # Process the segmentation results with SAM
        predictor = choose_model()
        results = predictor(color_img, points=center, labels=[1])
        center, mask = process_results(results)
      
        # Save the segmentation mask
        cv2.imwrite(output_path, mask)
    else:
      print("no valid target box")

    return color_img

def main():
    source = "p1.png"
    image = cv2.imread(source)
    segment_image(image)

if __name__ == "__main__":
    main()