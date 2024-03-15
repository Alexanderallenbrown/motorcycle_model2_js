//////// TEST ///////
var mbike = 100// kg, approx total weight of kx
var zbike = 0.7//m, height of rear frame CG no rider
var mrider = 70//kg, approx weight of rider
var zrider = 1.4 //m, approx height of CG of seated rider (1.09 seat + 0.3 to belly button)

var lam = 1.089
var a = .48//meters, distance from rear axle to CG in x direction
var hrf = (mbike*zbike+mrider*zrider)/(mbike+mrider)//meters, rear frame CG height
console.log("computed hrf: ",hrf)

var xp = 1.00 //distance to top of axis pivot
var zp = 1.15 //height of top of axis pivot
var Lf = 0.93//length of fork from top of clamp to axle
var ut = 0.02 //m, triple clamp offset
var ufx = 0.02//m, fork-axle offset in x direction
var mff = 15 //kg, fork mass
var Rfw = .68/2
var mfw = 15 //kg, wheel mass
var Rrw = .72/2 // radius of real wheel
var mrw = 15 //mass of rear wheel
var v = 15*.447; //m/s, fwd speed
var mrf = mbike+mrider - mff- mfw - mrw//kg, rear frame mass inc. rider
var bsteer = 0.0


/////eigenvalue plot
var eigPlot;
var rollChart;
var steerChart;
Chart.defaults.global.legend.labels.usePointStyle = true;


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

var moto_model = new MotorcycleModel(true,lam,a,xp,zp,ut,ufx,Lf,hrf,mrf,mff,Rfw,mfw,Rrw,mrw,bsteer)
var moto_model_ref = new MotorcycleModel(true,lam,a,xp,zp,ut,ufx,Lf,hrf,mrf,mff,Rfw,mfw,Rrw,mrw,bsteer)
var renderer;
// moto_model.updateModel(v)
var currVel = v;
var eigdata = moto_model.eigStudy(1,15,.1)
var eigdata_ref = moto_model.eigStudy(1,15,.1)
var stepdata = moto_model.stepResponse(currVel,1,15,.001)
var rolldata = stepdata[0]
var steerdata = stepdata[1]
var stepdata_ref = moto_model_ref.stepResponse(currVel,1,15,.001)
var rolldata_ref = stepdata_ref[0]
var steerdata_ref = stepdata_ref[1]

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
  var cw = canvasDiv.offsetWidth;
  var ch = canvasDiv.offsetHeight;
  myWidth = Math.min(cw,500);
  myHeight = Math.min(ch,350);
  console.log(myWidth,myHeight);
  var sketchCanvas = createCanvas(myWidth,myHeight);
  // console.log(sketchCanvas);
  sketchCanvas.parent("sketch-holder");
  initEigChart(eigdata, eigdata_ref, "eigenvalues as a function of speed; stable speeds shown in green", "speed (mph)", "eig (1/s)")
  initRollChart(rolldata,rolldata_ref,"Response to Sudden 1.0 N-m Handlebar Torque at "+(currVel/.447).toFixed()+" mph","Time (s)","Roll Angle (deg)")
  initSteerChart(steerdata,steerdata_ref,"Steer Step Response","Time (s)","Steer Angle (deg)")
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
  this.scale = myWidth/3
  this.origin_x = myWidth/1.5
  this.origin_y = 0.75*myHeight
  this.drawCG = function(minSize,sizeScale,x,y){

  }

  this.draw = function(){
    strokeWeight(1)
    //translate to the point where the rear wheel contacts ground
    translate(myWidth/2.0-this.model.b/2*this.scale,this.origin_y);
    // ellipse(0,0,10,10,10);
    //draw the ground
    strokeWeight(2)
    line(-this.model.Rrw*this.scale,0,(this.model.b+this.model.Rfw)*this.scale,0)
    ///draw front contact patch
    stroke(150,150,150)
    ellipse(this.model.b*this.scale,0,5,5)
    //draw trail intersect
    ellipse((this.model.b+this.model.c)*this.scale,0,5,5)
    stroke(0)
    //draw the frame
    push()
    strokeWeight(2)
    var pfendx=this.scale*(-this.model.Rrw);
    var pfendy = -this.scale*(this.model.zp+this.model.hrf)/2;
    var pgx = this.scale*(this.model.a)
    var pgy = -this.scale*(this.model.hrf);
    var pp2x = this.scale*(this.model.xp+0.2*Math.cos(this.model.lam))
    var pp2y = -this.scale*(this.model.zp-0.2*Math.sin(this.model.lam))
    var ppx = this.scale*this.model.xp;
    var ppy = -this.scale*this.model.zp;
    var pengx = this.scale*.87
    var pengy = -this.scale*.3
    var ppegx = this.scale*this.model.a
    var ppegy = -this.scale*.3
    var pfendlx = this.scale*(this.model.a)
    var pfendly = -this.scale*.65
    line(pfendx,pfendy,pgx,pgy);
    line(pfendx,pfendy,pfendlx,pfendly);
    line(pgx,pgy,ppx,ppy);
    line(pp2x,pp2y,pengx,pengy);
    line(pengx,pengy,ppegx,ppegy);
    line(ppegx,ppegy,pfendlx,pfendly);
    pop()
    //draw the rear wheel
    push()
    var rw_thick = 0.12
    strokeWeight(int(this.scale*rw_thick))
    translate(0,-(this.model.Rrw)*this.scale);
    ellipse(0,0,2*(this.model.Rrw-rw_thick/2)*this.scale,2*(this.model.Rrw-rw_thick/2)*this.scale)
    pop()
    //draw the front wheel
    push()
    var fw_thick = 0.08
    strokeWeight(int(this.scale*fw_thick))
    translate(this.model.b*this.scale,-(this.model.Rfw)*this.scale);
    ellipse(0,0,2*(this.model.Rfw-fw_thick/2)*this.scale,2*(this.model.Rfw-fw_thick/2)*this.scale)
    pop()
    //draw the steering axis
    push()
    strokeWeight(2)
    stroke(100,100,100)
    translate((this.model.b+this.model.c)*this.scale,0)
    // rotate(this.model.lam)
    line(0,0,-this.model.zp/Math.tan(this.model.lam)*this.scale,-this.model.zp*this.scale)
    pop()
    push()
    //draw the forks
    strokeWeight(10)
    stroke(0,0,0)
    translate(this.model.xp*this.scale+this.scale*this.model.ut/Math.cos(this.model.lam),-(this.model.zp+this.model.ut/Math.sin(lam))*this.scale)
    // rotate(-this.model.lam)
    line(0,0,this.scale*this.model.Lf*Math.cos(this.model.lam),this.scale*this.model.Lf*Math.sin(this.model.lam))
    pop()
    //draw the steering axis
    push()
    strokeWeight(2)
    stroke(100,100,100)
    translate((this.model.b+this.model.c)*this.scale,0)
    // rotate(this.model.lam)
    line(0,0,-this.model.zp/Math.tan(this.model.lam)*this.scale,-this.model.zp*this.scale,)
    pop()


    //draw the CG for rear frame
    push()
    stroke(0)
    var rcgdia = 30//+this.model.mrf/2
    translate(this.scale*this.model.a,-this.scale*this.model.hrf)
    fill(255)
    ellipse(0,0,rcgdia,rcgdia)
    fill(0)
    arc(0,0,rcgdia,rcgdia,0,PI/2,PIE)
    arc(0,0,rcgdia,rcgdia,PI,3*PI/2,PIE)
    pop()
    //draw the CG for frone frame
    push()
    var fcgdia = 30//+this.model.mff
    translate(this.scale*this.model.xff,-this.scale*this.model.zff)
    fill(255)
    ellipse(0,0,fcgdia,fcgdia)
    fill(0)
    arc(0,0,fcgdia,fcgdia,0,PI/2,PIE)
    arc(0,0,fcgdia,fcgdia,PI,3*PI/2,PIE)
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
    ]
  },
  options: {
    layout: {
            padding: 0
        },
    maintainAspectRatio: false,
    responsive: true,
    scales: {
      y:{
        min: -10,
        max:10,
      },
      yAxes: [{
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
          usePointStyle: true,
          pointStyleWidth: 1
        }
      }
    },

  }
};

eigPlot = new Chart(canv, config);



eigPlot.update();

return canv;
}

function initSteerChart(data, refdata, myTitle, xlabel, ylabel) {
  // console.log("In Steer Chart Function")
  canv = document.getElementById("steerchartCanvas");
  // console.log(canv)

  var config = {  // Chart.js configuration, including the DATASETS data from the model data
    type: "scatter",
    title: myTitle,

    data: {
      datasets: [{
        pointBackgroundColor: "rgba(0,0,0,1)",
        showLine: true,
        borderWidth: 0,
        borderColor: "rgba(0,0,0,1)",
        fill: false,
        label: 'current design',
        pointStyle: false,
        radius: 0,
        data: data,
      },
      {
          pointBackgroundColor: "rgba(100,100,100,.75)",
          showLine: true,
          borderColor: "rgba(100,100,100,.75)",
          borderWidth: 0,
          fill: false,
          label: 'reference (stock) design',
          pointStyle: false,
          radius: 0,
          data: refdata,
      }
    ]
  },
  options: {
    layout: {
            padding: {left: 0, right:-10, top:-30, bottom: -5}
        },
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
          usePointStyle: true,
          pointStyleWidth: 1
        }
      }
    },

  }
};

steerChart = new Chart(canv, config);

steerChart.update();

return canv;
}

function initRollChart(data, refdata, myTitle, xlabel, ylabel) {
  // console.log("In Roll Chart Function")
  canv = document.getElementById("rollchartCanvas");
  // console.log(canv)

  var config = {  // Chart.js configuration, including the DATASETS data from the model data
    type: "scatter",
    title: myTitle,

    data: {
      datasets: [{
        pointBackgroundColor: "rgba(0,0,0,1)",
        showLine: true,
        borderWidth: 0,
        borderColor: "rgba(0,0,0,1)",
        fill: false,
        label: 'current design',
        pointStyle: false,
        radius: 0,
        data: data,
      },
      {
          pointBackgroundColor: "rgba(100,100,100,.75)",
          showLine: true,
          borderColor: "rgba(100,100,100,.75)",
          borderWidth: 0,
          fill: false,
          label: 'reference (stock) design',
          pointStyle: false,
          radius: 0,
          data: refdata,
      }
    ]
  },
  options: {
    layout: {
            padding: {left: 0, right:-10, top:-10, bottom: -30}
        },
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
    }
  }
};

rollChart = new Chart(canv, config);
// console.log(rollChart)

rollChart.update();

return canv;
}



updateEigChart = function(){
  canv = document.getElementById("eigchartCanvas");
  //update eig data
  eigdata = moto_model.eigStudy(1,15,.1)
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
  //do another step response and update those plots too
  stepdata = moto_model.stepResponse(currVel,1,15,.001)
  stepdata_ref = moto_model_ref.stepResponse(currVel,1,15,.001)
  rollChart.data.datasets[0].data = stepdata[0]
  steerChart.data.datasets[0].data = stepdata[1]
  rollChart.data.datasets[1].data = stepdata_ref[0]
  steerChart.data.datasets[1].data = stepdata_ref[1]
  rollChart.update()
  steerChart.update()
  renderer.model = moto_model
}


ut_slider.oninput = function(){
  ut = this.value/1000.0;
  moto_model.ut = ut
  document.getElementById("ut_sliderval").innerHTML = str(ut*1000.0)
  document.getElementById("sensval").innerHTML = moto_model.sensitivity_steer
  document.getElementById("wheelbaseval").innerHTML = (moto_model.b/0.0254).toFixed(2)
  document.getElementById("trailval").innerHTML = (moto_model.c/0.0254).toFixed(2)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

lam_slider.oninput = function(){
  rake = this.value/10.0*3.1415/180.0
  lam = 3.1415/2.0-rake;
  moto_model.lam = lam
  document.getElementById("lam_sliderval").innerHTML = (rake*180/3.1415).toFixed()
  document.getElementById("sensval").innerHTML = moto_model.sensitivity_steer
  document.getElementById("wheelbaseval").innerHTML = (moto_model.b/0.0254).toFixed(2)
  document.getElementById("trailval").innerHTML = (moto_model.c/0.0254).toFixed(2)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

v_slider.oninput = function(){
  v = this.value/10.0*0.447;
  currVel = v;
  rollChart.options.title.text = "Response to Sudden 1.0 Nm Handlebar Torque at "+(currVel/.447).toFixed()+" mph"
  moto_model.v = v
  document.getElementById("v_sliderval").innerHTML = ((1.0/.447)*v).toFixed()
  document.getElementById("sensval").innerHTML = moto_model.sensitivity_steer
  document.getElementById("wheelbaseval").innerHTML = (moto_model.b/0.0254).toFixed(2)
  document.getElementById("trailval").innerHTML = (moto_model.c/0.0254).toFixed(2)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

// bsteer_slider.oninput = function(){
//   bsteer = this.value/100.0;
//   moto_model.bsteer = bsteer
//   document.getElementById("bsteer_sliderval").innerHTML = str(bsteer)
//   // print(moto_model.hrf)
//   updateEigChart()
//   //print("wheel pos udpate: " +str(this.value))
// }
