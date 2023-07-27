import pyrealsense2 as rs

def get_serial_number():
    try:
        context = rs.context()
        devices = context.query_devices()
        if len(devices) == 0:
            raise Exception("No RealSense devices found.")
        else:
            serial_number = devices[0].get_info(rs.camera_info.serial_number)
            return serial_number
    except Exception as e:
        print("Error:", e)
        return None
    
if __name__ == "__main__":
    print(f"serial number : {get_serial_number()}")