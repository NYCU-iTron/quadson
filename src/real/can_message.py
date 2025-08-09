class CanMessage:
    def __init__(self, motor_id, id_type, msg_id, data):
        self.motor_id = motor_id
        self.id_type = id_type
        self.msg_id = msg_id 
        self.data = data
    
    def to_string(self):
        return f"Motor ID: {self.motor_id}, ID Type: {self.id_type}, Message ID: {self.msg_id}, Data: {self.data}"
        