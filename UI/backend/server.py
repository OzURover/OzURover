from flask import Flask, render_template

app = Flask(
    __name__,
    static_folder="../rover-ui/build/static",
    template_folder="../rover-ui/build",
)

@app.route("/")
def index():
    return render_template("index.html")


app.run(debug=True)