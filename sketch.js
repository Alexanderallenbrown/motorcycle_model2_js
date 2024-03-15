//////// TEST ///////
var mbike = 100// kg, approx total weight of kx
var zbike = 0.7//m, height of rear frame CG no rider
var mrider = 70//kg, approx weight of rider
var zrider = 1.4 //m, approx height of CG of seated rider (1.09 seat + 0.3 to belly button)

var lam = 1.089
var a = .48//meters, distance from rear axle to CG in x direction
var hrf = (mbike*zbike+mrider*zrider)/(mbike+mrider)//meters, rear frame CG height
console.log("computed hrf: ",hrf)

var xp = 1.09 //distance to top of axis pivot
var zp = 1.15 //height of top of axis pivot
var Lf = 0.93//length of fork from top of clamp to axle
var ut = 0.02 //m, triple clamp offset
var ufx = 0.02//m, fork-axle offset in x direction
var mff = 15 //kg, fork mass
var Rfw = .68/2
var mfw = 15 //kg, wheel mass
var Rrw = .72/2 // radius of real wheel
var mrw = 15 //mass of rear wheel
var v = 4; //m/s, fwd speed
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
var currVel = 4;
var eigdata = moto_model.eigStudy(1,10,.1)
var eigdata_ref = moto_model.eigStudy(1,10,.1)
var stepdata = moto_model.stepResponse(currVel,.1,5,.001)
var rolldata = stepdata[0]
var steerdata = stepdata[1]
var stepdata_ref = moto_model_ref.stepResponse(currVel,.1,5,.001)
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
  myWidth = canvasDiv.offsetWidth;
  myHeight = 300//canvasDiv.offsetHeight;
  var sketchCanvas = createCanvas(myWidth,myHeight);
  // console.log(sketchCanvas);
  sketchCanvas.parent("sketch-holder");
  initEigChart(eigdata, eigdata_ref, "eigenvalues as a function of speed; stable speeds shown in green", "speed (m/s)", "eig (1/s)")
  initRollChart(rolldata,rolldata_ref,"Response to 0.1 Nm Step in Handlebar Torque at "+str(currVel)+" m/s","Time (s)","Roll Angle (deg)")
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
  this.scale = myWidth/1.5
  this.origin_x = myWidth/1.5
  this.origin_y = 0.75*myHeight
  this.drawCG = function(minSize,sizeScale,x,y){

  }

  this.draw = function(){
    strokeWeight(1)
    //translate to the point where the rear wheel contacts ground
    translate(myWidth/2.0-this.model.b/2*this.scale,this.origin_y);
    // ellipse(0,0,10,10,10);
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
    strokeWeight(2)
    translate((this.model.b+this.model.c)*this.scale,0)
    rotate(this.model.lam)
    line(0,0,-3*this.model.Rfw*this.scale,0)
    pop()
    //draw the CG for rear frame
    push()
    var rcgdia = 10+this.model.mrf/2
    translate(this.scale*this.model.a,-this.scale*this.model.hrf)
    fill(255)
    ellipse(0,0,rcgdia,rcgdia)
    fill(0)
    arc(0,0,rcgdia,rcgdia,0,PI/2,PIE)
    arc(0,0,rcgdia,rcgdia,PI,3*PI/2,PIE)
    pop()
    //draw the CG for frone frame
    push()
    var fcgdia = 10+this.model.mff
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
    layout: {
            padding: 0
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
          label: 'reference design',
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
          label: 'reference design',
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
  //do another step response and update those plots too
  stepdata = moto_model.stepResponse(currVel,.1,5,.001)
  stepdata_ref = moto_model_ref.stepResponse(currVel,.1,5,.001)
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
  currVel = v;
  rollChart.options.title.text = "Response to 0.1 Nm Step in Handlebar Torque at "+str(currVel)+" m/s"
  moto_model.v = v
  document.getElementById("v_sliderval").innerHTML = str(v)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}

bsteer_slider.oninput = function(){
  bsteer = this.value/100.0;
  moto_model.bsteer = bsteer
  document.getElementById("bsteer_sliderval").innerHTML = str(bsteer)
  // print(moto_model.hrf)
  updateEigChart()
  //print("wheel pos udpate: " +str(this.value))
}
