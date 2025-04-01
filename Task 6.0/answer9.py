import cv2 as cv
import numpy as np

yellow_team_count = 0
red_team_count = 0

def find_ball(frame, movement_mask):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    lower_bound = np.array([20, 100, 100])
    upper_bound = np.array([40, 255, 255])

    mask = cv.inRange(hsv, lower_bound, upper_bound)

    
    combined_mask = cv.bitwise_and(mask, movement_mask)
    cv.imwrite("sdjshc.jpg",combined_mask)
    kernel = np.ones((5, 5), np.uint8)
    combined_mask = cv.dilate(combined_mask, kernel, iterations=2)
    combined_mask = cv.erode(combined_mask, kernel, iterations=1)

    contours, _ = cv.findContours(combined_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    largest_radius = 0
    ball_info = None








    for c in contours:
        area = cv.contourArea(c)
        if area < 100 or area > 3000:  
            continue

        ((x, y), radius) = cv.minEnclosingCircle(c)
        circularity = (4 * np.pi * area) / (cv.arcLength(c, True) ** 2 + 1e-5)

        if 0.7 <= circularity <= 1.2 and radius > largest_radius:  
            largest_radius = radius
            ball_info = (int(x), int(y), int(radius))

    if ball_info:
        x, y, radius = ball_info
        #print("Ball detected at (", x, ",", y, ")")

        cv.circle(frame, (x, y), radius, (0, 255, 0), 2)

        return [x, y]

    return []






def identify_players(frame):
    global yellow_team_count, red_team_count
    
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    yellow_lower = np.array([5, 0, 100])
    yellow_upper = np.array([67, 255, 255])
    red_lower = np.array([175, 137, 100])
    red_upper = np.array([178, 255, 255])
    
    yellow_mask = cv.inRange(hsv, yellow_lower, yellow_upper)
    red_mask = cv.inRange(hsv, red_lower, red_upper)
    
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    
    yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_OPEN, kernel)
    red_mask = cv.morphologyEx(red_mask, cv.MORPH_OPEN, kernel)
    
    yellow_contours, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    yellow_count = 0
    red_count = 0
    




    for cnt in yellow_contours:
        if cv.contourArea(cnt) > 500:
            x, y, w, h = cv.boundingRect(cnt)
            center = (x + w // 2, y + h // 2)
            cv.circle(frame, center, 5, (0, 255, 255), -1)
            yellow_count += 1
    


    for cnt in red_contours:
        if cv.contourArea(cnt) > 500:
            x, y, w, h = cv.boundingRect(cnt)
            center = (x + w // 2, y + h // 2)
            cv.circle(frame, center, 5, (0, 0, 255), -1)
            red_count += 1
    





    yellow_team_count += yellow_count
    red_team_count += red_count
    
    return yellow_count, red_count





def capaing(video_path, output_path):

    

    cap = cv.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error opening video file")
        return

    frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv.CAP_PROP_FPS))
    total_frames = int(cap.get(cv.CAP_PROP_FRAME_COUNT))

    print(f"Processing {video_path}: {total_frames} frames, {fps} FPS, {frame_width}x{frame_height}")

    out = cv.VideoWriter(output_path, cv.VideoWriter_fourcc(*'mp4v'), fps, (frame_width, frame_height))

    frame_count = 0

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            print("End of video reached!")
            break

        frame_count += 1

        movement_mask = np.ones_like(frame[:, :, 0], dtype=np.uint8) * 255

        ball_position = find_ball(frame, movement_mask)
        yellow_players, red_players = identify_players(frame)

        cv.putText(frame, f"Yellow Team: {yellow_players}", (50, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv.putText(frame, f"Red Team: {red_players}", (70, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if frame_count % 50 == 0:
            print("-" * 50)
            print(f"Processed {frame_count}/{total_frames} frames ({(frame_count / total_frames) * 100:.1f}%)")

        cv.imshow('Ball Tracking', frame)
        out.write(frame)

        if cv.waitKey(25) & 0xFF == ord('q'):
            print("User interrupted processing!")
            break

    print("Processing done!")

    cap.release()
    out.release()
    cv.destroyAllWindows()

video_path = 'volleyball_match.mp4'
output_video_path = 'I am atomic.mp4'

capaing(video_path, output_video_path)


