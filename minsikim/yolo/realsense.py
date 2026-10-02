import pyrealsense2 as rs, numpy as np, cv2

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)
try:
    while True:
        f = pipe.wait_for_frames()
        c, d = f.get_color_frame(), f.get_depth_frame()
        if not c or not d:
            continue
        color = np.asanyarray(c.get_data())
        depth = np.asanyarray(d.get_data())
        depth_vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.05), cv2.COLORMAP_JET)
        cv2.imshow("RealSense", np.hstack((color, depth_vis)))
        if cv2.waitKey(1) == 27:  # ESC 종료
            break
finally:
    pipe.stop()
    cv2.destroyAllWindows()