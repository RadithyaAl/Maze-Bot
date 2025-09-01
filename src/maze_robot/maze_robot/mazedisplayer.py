import rclpy
import random
import time
from rclpy.node import Node
from robot_interfaces.srv import MazePath


class MazeDisplayer(Node):

    def __init__(self, width, height):
        super().__init__('maze_displayer')
        self.width = width
        self.height = height
        self.grid = self.generate_maze()
        self.cli = self.create_client(MazePath, 'solve_maze')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def generate_maze(self):
        grid = [['#' for _ in range(self.width)] for _ in range(self.height)]

        def carve_passages(x, y):
            directions = [(0, 2), (2, 0), (0, -2), (-2, 0)]
            random.shuffle(directions)
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.height and 0 <= ny < self.width and grid[nx][ny] == '#':
                    grid[x + dx // 2][y + dy // 2] = ' '
                    grid[nx][ny] = ' '
                    carve_passages(nx, ny)

        grid[1][1] = ' '
        carve_passages(1, 1)
        return grid

    def display(self, grid=None):
        if grid is None:
            grid = self.grid
        for row in grid:
            print(''.join(row))

    def send_request(self, grid):
        req = MazePath.Request()
        req.maze = [''.join(row) for row in grid]

        self.future = self.cli.call_async(req)

        return self.future


def main(args=None):
    rclpy.init(args=args)
    node = MazeDisplayer(75, 35)

    print("\nMaze asli:")
    node.display()
    current_grid = [row[:] for row in node.grid]  # copy grid asli

    node.send_request(node.grid)
    rclpy.spin_until_future_complete(node, node.future)

    if node.future.result() is not None:
        res = node.future.result()
        flat_path = res.path

        # Convert back to list of tuples
        path = [(flat_path[i], flat_path[i+1]) for i in range(0, len(flat_path), 2)]

        for i in range(len(path)):
            x, y = path[i]
            current_grid[x][y] = '*'

            # clear screen biar terlihat update
            print("\033c", end="")  
            node.display(current_grid)

            time.sleep(0.2)

    else:
        node.get_logger().error('Service call failed!')

    rclpy.shutdown()



if __name__ == '__main__':
    main()
