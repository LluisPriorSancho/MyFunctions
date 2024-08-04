import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from example_interfaces.srv import AddTwoInts
from rclpy.executors import MultiThreadedExecutor

class MyCheatsheet_Pub_Sub_Timer(Node):
    def __init__(self):
        super().__init__('MyCheatsheet_Pub_Sub_Timer')
        
        # Publisher
        self.publisher_ = self.create_publisher(Empty, '/my_cheatsheet/publisher', 10)
        
        # Subscriber
        self.subscriber_ = self.create_subscription(Empty, '/my_cheatsheet/publisher', self.subscriber_callback, 10)
        
        # Timer
        self.timer_ = self.create_timer(1, self.on_timer)
        
    # Subscriber callback
    def subscriber_callback(self, msg: Empty):
        self.get_logger().info('My_cheatsheet is working!')
        
    # Timer callback
    def on_timer(self):
        # Publishing message
        self.publisher_.publish(msg = Empty())
        
class MyCheatsheet_Service_Server(Node):
    def __init__(self):
        super().__init__('MyCheatsheet_Service_Server')
        
        # Service server
        self.srv = self.create_service(AddTwoInts, 'my_cheatsheet/service', self.service_callback)

    # Service callback
    def service_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response
        
class MyCheatsheet_Service_Client(Node):
    def __init__(self):
        super().__init__('MyCheatsheet_Service_Client')
        
        # Service client
        self.cli = self.create_client(AddTwoInts, 'my_cheatsheet/service')
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        # Request message
        self.req = AddTwoInts.Request()
        
        # Send request and wait response
        future = self.send_request(1, 2)
        future.add_done_callback(self.service_response_callback)
        
    # Response callback
    def service_response_callback(self, future):
        response = future.result()
        self.get_logger().info('Server response is ' + str(response))

    # Request function
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    try:
        MyCheatsheet_Pub_Sub_Timer_node = MyCheatsheet_Pub_Sub_Timer()
        MyCheatsheet_Service_Server_node = MyCheatsheet_Service_Server()
        MyCheatsheet_Service_Client_node = MyCheatsheet_Service_Client()
        
        executor = MultiThreadedExecutor()
        executor.add_node(MyCheatsheet_Pub_Sub_Timer_node)
        executor.add_node(MyCheatsheet_Service_Server_node)
        executor.add_node(MyCheatsheet_Service_Client_node)
        
        try: 
            executor.spin()
        finally:
            executor.shutdown()
            MyCheatsheet_Pub_Sub_Timer_node.destroy_node()
            MyCheatsheet_Service_Server_node.destroy_node()
            MyCheatsheet_Service_Client_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()