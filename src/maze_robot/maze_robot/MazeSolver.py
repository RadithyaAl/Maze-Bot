import rclpy
from rclpy.node import Node
from robot_interfaces.srv import MazePath


class MazeSolverNode(Node):

    def __init__(self):
        super().__init__('maze_solver')
        self.srv = self.create_service(MazePath, 'solve_maze', self.solve_callback)

    def solve_callback(self, request, response):
        grid = [list(row) for row in request.maze]
        height, width = len(grid), len(grid[0])
        start = (1, 1)
        goal = (height - 2, width - 2)
        path = []

        def deepfirstsearch(x, y):
            if (x, y) == goal:
                path.append((x, y))
                return True
            if grid[x][y] != ' ' and (x, y) != start:
                return False
            grid[x][y] = '.' # tandai dikunjungi

            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < height and 0 <= ny < width:
                    if deepfirstsearch(nx, ny):
                        path.append((x, y))
                        return True

        deepfirstsearch(*start)
        
        path = path[::-1]
        response.path = []
        for (x, y) in path:
            response.path.extend([x, y])  # flatten into [x, y]

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
