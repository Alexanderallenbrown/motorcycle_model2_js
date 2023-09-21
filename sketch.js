//////// TEST ///////
var lam = 1.13
var hrf = .25606
var a = .3386//meters, distance from rear axle to CG in x direction
var b = .767//meters, wheelbase of bike
var c = .023//.08//meters, trail
var hrf = .25606//meters, rear frame CG height
var mr = 11.065//kg, rear frame mass inc. rider
var xff = .62218//position of front frame CG
var yff = 0
var zff = .46531
var mff = 2.2047 //kg, fork mass
var Rfw = .15875
var mfw = 1.486 //kg, wheel mass
var Rrw = 0.15875 // radius of real wheel
var mrw = 2.462 //mass of rear wheel
var v = 4 //m/s, fwd speed

/////eigenvalue plot
var eigPlot;

////////INTERACTIVE ELEMENTS:

var a_slider = document.getElementById("a_slider");
var h_slider = document.getElementById("h_slider");
var mr_slider = document.getElementById("mr_slider");
var xf_slider = document.getElementById("xf_slider");
var zf_slider = document.getElementById("zf_slider");
var mf_slider = document.getElementById("mf_slider");
var Rfw_slider = document.getElementById("Rfw_slider");
var mfw_slider = document.getElementById("mfw_slider");
var Rrw_slider = document.getElementById("Rrw_slider");
var mrw_slider = document.getElementById("mrw_slider");
var c_slider = document.getElementById("c_slider");
var b_slider = document.getElementById("b_slider");
var lam_slider = document.getElementById("lam_slider");
var v_slider = document.getElementById("v_slider");

var moto_model = new MotorcycleModel(true,lam,a,b,c,hrf,mr,xff,yff,zff,mff,Rfw,mfw,Rrw,mrw)
var renderer;
// moto_model.updateModel(v)
var eigdata = moto_model.eigStudy(1,10,.1)
var eigdata_ref = moto_model.eigStudy(1,10,.1)

// var inconsolata;
// function preload() {
//   inconsolata = loadFont('assets/inconsolata.ttf');
// }

var myWidth;
var myHeight;
var canvasDiv;

function setup() {
  //createCanvas(400, 400);
  canvasDiv = document.getElementById('sketch-holder');
  myWidth = canvasDiv.offsetWidth;
  myHeight = 300//canvasDiv.offsetHeight;
  var sketchCanvas = createCanvas(myWidth,myHeight);
  console.log(sketchCanvas);
  sketchCanvas.parent("sketch-holder");
  initEigChart(eigdata, eigdata_ref, "eigenvalues as a function of speed; stable speeds shown in green", "speed (m/s)", "eig (1/s)")
  renderer = new modelRenderer(moto_model)
}

function draw() {
  background(255);
  stroke(0);
  noFill();
  renderer.draw();
}

function modelRenderer(motomodel){
  this.model = motomodel
  this.scale = myWidth/1.5
  this.origin_x = myWidth/1.5
  this.origin_y = 0.75*myHeight
  this.drawCG = function(minSize,sizeScale,x,y){

  }

  this.draw = function(){
    strokeWeight(1)
    //translate to the point where the rear wheel contacts ground
    translate(myWidth/2.0-this.model.b/2*this.scale,this.origin_y);
    ellipse(0,0,10,10,10);
    //draw the frame
    push()
    strokeWeight(2)
    translate(0,-this.model.Rrw*this.scale)
    line(0,0,this.scale*(this.model.b-this.model.Rfw)/3,0)
    push()
    rotate(-PI/4)
    line(0,0,this.scale*(this.model.b-this.model.Rfw)/3/.707,0)
    pop()
    line(this.scale*(this.model.b-this.model.Rfw)/3,0,this.scale*(this.model.b-this.model.Rfw)/3,-(this.model.b-this.model.Rfw)/3*this.scale)
    rect(this.scale*(this.model.b-this.model.Rfw)/3,0,this.scale*(this.model.b-this.model.Rfw)/3,-this.scale*(this.model.b-this.model.Rfw)/3)
    translate(this.scale*(this.model.b-this.model.Rfw)*2.0/3,0)
    triangle(0,0,0,-this.scale*(this.model.b-this.model.Rfw)/3,this.scale*(this.model.b-this.model.Rfw)/3,-this.scale*(this.model.b-this.model.Rfw)/3)
    pop()
    //draw the rear wheel
    push()
    strokeWeight(int(this.model.mrw)*3+1)
    translate(0,-this.model.Rrw*this.scale);
    ellipse(0,0,2*this.model.Rrw*this.scale,2*this.model.Rrw*this.scale)
    pop()
    //draw the front wheel
    push()
    strokeWeight(int(this.model.mfw)*3+1)
    translate(this.model.b*this.scale,-this.model.Rfw*this.scale);
    ellipse(0,0,2*this.model.Rfw*this.scale,2*this.model.Rfw*this.scale)
    pop()
    //draw the steering axis
    push()
    strokeWeight(3)
    translate((this.model.b+this.model.c)*this.scale,0)
    rotate(this.model.lam)
    line(0,0,-3*this.model.Rfw*this.scale,0)
    pop()
  }
}


/////////////// PLOT PLOTTING plot plotting make chart
function initEigChart(data, refdata, myTitle, xlabel, ylabel) {
  canv = document.getElementById("eigchartCanvas");

  //point colors to indicate stability:
  var pointBGColors_active = []
  // console.log("starting color thing")
  for (i = 0; i < data[3].length; i++) {
    // console.log(data[3][i])
    if (data[3][i]==true) {
      pointBGColors_active.push("rgba(0,125,0,1)");
      // console.log("STABLE!!")
    } else {
      pointBGColors_active.push("rgba(0,0,0,1)");
    }
  }

  //point colors to indicate stability:
  var pointBGColors_ref = []
  // console.log("starting color thing")
  for (i = 0; i < data[3].length; i++) {
    // console.log(data[3][i])
    if (data[3][i]==true) {
      pointBGColors_ref.push("rgba(0,125,0,.25)");
      // console.log("STABLE!!")
    } else {
      pointBGColors_ref.push("rgba(0,0,0,.25)");
    }
  }

  var config = {  // Chart.js configuration, including the DATASETS data from the model data
    type: "scatter",
    title: myTitle,

    data: {
      datasets: [{
        // xAxisID: "Time (s)",
        // yAxisID: "Output",
        pointBackgroundColor: pointBGColors_active,
        showLine: false,
        borderColor: pointBGColors_active,
        fill: true,
        label: 'real',
        pointStyle: 'circle',
        data: data[1],
      },
      {
        // xAxisID: "Time (s)",
        // yAxisID: "Output",
        pointBackgroundColor: pointBGColors_active,
        showLine: false,
        borderColor: pointBGColors_active,
        fill: true,
        label: 'imaginary',
        pointStyle: 'cross',
        data: data[2]
      }
      // },
      // {
      //     // xAxisID: "Time (s)",
      //     // yAxisID: "Output",
      //     pointBackgroundColor: pointBGColors_ref,
      //     showLine: false,
      //     borderColor: pointBGColors_ref,
      //     fill: true,
      //     label: 'real',
      //     pointStyle: 'circle',
      //     data: refdata[1],
      // },
      // {
      //     // xAxisID: "Time (s)",
      //     // yAxisID: "Output",
      //     pointBackgroundColor: pointBGColors_ref,
      //     showLine: false,
      //     borderColor: pointBGColors_ref,
      //     fill: true,
      //     label: 'imaginary',
      //     pointStyle: 'cross',
      //     data: refdata[2]
      // }
    ]
  },
  options: {
    maintainAspectRatio: false,
    responsive: true,
    scales: {
      yAxes: [{
        min: -10,
        max: 10,
        scaleLabel: {
          display: true,
          labelString: (ylabel),
        }
      }],
      xAxes: [{
        scaleLabel: {
          display: true,
          labelString: (xlabel),
        }
      }]
    },
    title: {
      display: true,
      text: myTitle
    },
    plugins: {
      legend: {
        labels: {
          usePointStyle: true
        }
      }
    },

  }
};

eigPlot = new Chart(canv, config);



eigPlot.update();

return canv;
}

updateEigChart = function(){
  canv = document.getElementById("eigchartCanvas");
  //update eig data
  eigdata = moto_model.eigStudy(1,10,.1)
  eigPlot.data.datasets[0].data = eigdata[1]
  eigPlot.data.datasets[1].data = eigdata[2]
  //point colors to indicate stability:
  var pointBGColors_active = []
  // console.log("starting color thing")
  for (i = 0; i < eigdata[3].length; i++) {
    // console.log(eigdata[3][i])
    if (eigdata[3][i]==true) {
      pointBGColors_active.push("rgba(0,125,0,1)");
      // console.log("STABLE!!")
    } else {
      pointBGColors_active.push("rgba(0,0,0,1)");
    }
  }
  eigPlot.data.datasets[0].pointBackgroundColor = pointBGColors_active
  eigPlot.data.datasets[0].borderColor = pointBGColors_active
  eigPlot.data.datasets[1].pointBackgroundColor = pointBGColors_active
  eigPlot.data.datasets[1].borderColor = pointBGColors_active
  eigPlot.update()
  renderer.model = moto_model
}


a_slider.oninput = function(){
  a = this.value/1000.0;
  moto_model.a = a
  document.getElementById("a_sliderval").innerHTML = str(a)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

h_slider.oninput = function(){
  h = this.value/1000.0;
  moto_model.hrf = h
  document.getElementById("h_sliderval").innerHTML = str(h)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

mr_slider.oninput = function(){
  mr = this.value/10.0;
  moto_model.mrf = mr
  document.getElementById("mr_sliderval").innerHTML = str(mr)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

xf_slider.oninput = function(){
  xf = this.value/1000.0;
  moto_model.xff = xf+moto_model.b
  document.getElementById("xf_sliderval").innerHTML = str(xf)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

zf_slider.oninput = function(){
  zf = this.value/1000.0;
  moto_model.zf = zf
  document.getElementById("zf_sliderval").innerHTML = str(zf)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

mf_slider.oninput = function(){
  mf = this.value/100.0;
  moto_model.mff = mf
  document.getElementById("mf_sliderval").innerHTML = str(mf)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

Rfw_slider.oninput = function(){
  Rfw = this.value/1000.0;
  moto_model.Rfw = Rfw
  document.getElementById("Rfw_sliderval").innerHTML = str(Rfw)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

mfw_slider.oninput = function(){
  mfw = this.value/10.0;
  moto_model.mfw = mfw
  document.getElementById("mfw_sliderval").innerHTML = str(mfw)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

Rrw_slider.oninput = function(){
  Rrw = this.value/1000.0;
  moto_model.Rrw = Rrw
  document.getElementById("Rrw_sliderval").innerHTML = str(Rrw)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

mrw_slider.oninput = function(){
  mrw = this.value/10.0;
  moto_model.mrw = mrw
  document.getElementById("mrw_sliderval").innerHTML = str(mrw)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

b_slider.oninput = function(){
  delta = moto_model.xf-moto_model.b
  b = this.value/1000.0;
  moto_model.b = b
  moto_model.xf = b+delta
  document.getElementById("b_sliderval").innerHTML = str(b)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

c_slider.oninput = function(){
  c = this.value/10000.0;
  moto_model.c = c
  document.getElementById("c_sliderval").innerHTML = str(c)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

lam_slider.oninput = function(){
  rake = this.value/10.0*3.1415/180.0
  lam = 3.1415/2.0-rake;
  moto_model.lam = lam
  document.getElementById("lam_sliderval").innerHTML = (rake*180/3.1415).toFixed()
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

v_slider.oninput = function(){
  v = this.value/10.0;
  moto_model.v = v
  document.getElementById("v_sliderval").innerHTML = str(v)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}
