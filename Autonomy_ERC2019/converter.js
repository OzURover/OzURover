const XLSX = require("xlsx");
const path = require("path");
const fs = require("fs");

var world = [];

const directoryPath = path.join(__dirname, "bin");
var worksheet = null;

fs.readdir(directoryPath, function(err, files) {
  files.forEach(function(file) {
    if (file == ".DS_Store") return false;
    var workbook = XLSX.readFile(path.join(directoryPath, file));
    var data = workbook.SheetNames[0];
    worksheet = workbook.Sheets[data];
    world += "x, y, z\n";

    for (var ii = 1; cell(1, ii) != undefined; ii++) {
      for (var i = 1; cell(i, ii) != undefined; i++) {
        val = cell(i, ii); 
        if (val == 0) {
          val = 1;
        }
        world += i/10 + "," + ii/10 + "," + val + "\n";
      }
    }

    fs.writeFile("DTM.csv", world, err => {
      console.log(err);
    });
  });
});

function cell(i, ii) {
  name = toColumnName(i) + ii;
  var desired_cell = worksheet[name];
  return desired_cell ? desired_cell.v : undefined;
}

function toColumnName(num) {
  for (var ret = "", a = 1, b = 26; (num -= a) >= 0; a = b, b *= 26) {
    ret = String.fromCharCode(parseInt((num % b) / a) + 65) + ret;
  }
  return ret;
}
