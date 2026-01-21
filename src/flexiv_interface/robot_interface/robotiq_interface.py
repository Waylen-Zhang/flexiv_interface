import threading
import queue
import time
from pyrobotiqgripper import RobotiqGripper

class SafeGripperController:
    def __init__(self):
        self.gripper = RobotiqGripper()
        self.gripper.activate()
        self.gripper.calibrate(0, 40)
        self.command_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.running = True
        
        self.process_thread = threading.Thread(
            target=self._command_processor, 
            daemon=True
        )
        self.process_thread.start()
        self._setup_gripper()
    
    def _setup_gripper(self):
        if hasattr(self.gripper, 'instrument'):
            self.gripper.instrument.serial.timeout = 0.5
            self.gripper.instrument.serial.write_timeout = 0.5
            self.gripper.instrument.clear_buffers_before_each_call = True
    
    def _command_processor(self):
        while self.running:
            try:
                command, args, kwargs = self.command_queue.get(timeout=0.1)
                
                try:
                    result = command(*args, **kwargs)
                    self.result_queue.put(('success', result))
                except Exception as e:
                    error_msg = str(e)
                    if any(term in error_msg for term in [
                        'Checksum error', 
                        'readiness to read',
                        'no data',
                        'device disconnected'
                    ]):
                        self.result_queue.put(('ignored_error', None))
                    else:
                        self.result_queue.put(('error', e))
                
                self.command_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"命令处理器异常: {e}")
                time.sleep(0.1)
    
    def _submit_command(self, command, *args, **kwargs):
        self.command_queue.put((command, args, kwargs))
    
    def _wait_for_result(self, timeout=2.0):
        try:
            status, result = self.result_queue.get(timeout=timeout)
            if status == 'error':
                raise result
            return result
        except queue.Empty:
            return None
    
    def open_gripper(self, force=0, speed=255):
        self._submit_command(self.gripper.open, force, speed)
    
    def close_gripper(self, force=255, speed=255):
        self._submit_command(self.gripper.close, force, speed)
    
    def move_gripper(self, position, force=255, speed=255):
        self._submit_command(self.gripper.goTo, position, force, speed)
    
    def stop(self):
        self.running = False
        self.process_thread.join(timeout=1.0)

    def get_gripper_distance(self):
        # open: maximum 40mm, close: minimum 0mm
        if hasattr(self, 'gripper'):
            return self.gripper.getPositionmm()
        else:
            return None
        
    def get_gripper_state(self):
        # return 1 for closed, 0 for opened
        if hasattr(self, 'gripper'):
            if self.gripper.getPositionmm() < 35.0:
                return 1
            elif self.gripper.getPositionmm() > 35.0:
                return 0
        else:
            return None