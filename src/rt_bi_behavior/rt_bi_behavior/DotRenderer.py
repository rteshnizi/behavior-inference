import rclpy
from flask import Flask, render_template_string
from rclpy.node import Node

flaskApp = Flask("BA_RENDERER")

htmlTemplate = """<!DOCTYPE html>
<meta charset="utf-8">
<body>
<script src="https://d3js.org/d3.v7.js"></script>
<script src="https://unpkg.com/@hpcc-js/wasm@2/dist/graphviz.umd.js" type="javascript/worker"></script>
<script src="https://unpkg.com/d3-graphviz@5/build/d3-graphviz.js"></script>
<div id="graph" style="text-align: center;"></div>
<script>

  d3.select("#graph").graphviz({useWorker: true})
    .renderDot('digraph  {a -> b}');

</script>
"""

@flaskApp.route("/")
def serve_html():
	generate_dot()
	return render_template_string(htmlTemplate)

def generate_dot():
	pass

class DotRenderer(Node):
	def __init__(self) -> None:
		flaskApp.run(host="0.0.0.0")
		return

	def destroy_node(self) -> None:
		super().destroy_node()
		return

def main(args=None):
	rclpy.init(args=args)
	renderer = DotRenderer()
	rclpy.spin(renderer)
	renderer.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
