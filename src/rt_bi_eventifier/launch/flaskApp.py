from pathlib import Path

from flask import Flask, render_template_string, send_from_directory

templateDir = Path(Path(__file__).parent).resolve()
templatePath = Path(str(templateDir), "graphviz.html")

App = Flask("BA_RENDERER")

@App.route("/")
def renderTemplate():
	print(f"Flask is rendering {str(templatePath)}.")
	htmlTemplate = templatePath.read_text()
	return render_template_string(htmlTemplate)

@App.route("/favicon.ico")
def favIcon():
	return send_from_directory(
		str(templateDir),
		"favicon.ico",
		mimetype="image/vnd.microsoft.icon"
	)
