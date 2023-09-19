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

var moto_model = new MotorcycleModel(true,lam,a,b,c,hrf,mr,xff,yff,zff,mff,Rfw,mfw,Rrw,mrw)
// moto_model.updateModel(v)
var eigdata = moto_model.eigStudy(1,10,.1)

// var inconsolata;
// function preload() {
//   inconsolata = loadFont('assets/inconsolata.ttf');
// }



function setup() {
  createCanvas(800, 400,WEBGL);
  initEigChart(eigdata, eigdata, "eigenvalues as a function of speed (all real parts must be <0 for stability)", "speed (m/s)", "eig (1/s)")
}

function draw() {
  background(255);
}

/////////////// PLOT PLOTTING plot plotting make chart
function initEigChart(data, refdata, myTitle, xlabel, ylabel) {
        canv = document.getElementById("eigchartCanvas");

        //point colors to indicate stability:
        var pointBGColors_active = []
        // console.log("starting color thing")
        for (i = 0; i < data[3].length; i++) {
          console.log(data[3][i])
          if (data[3][i]==true) {
              pointBGColors_active.push("rgba(0,125,0,1)");
              // console.log("STABLE!!")
          } else {
              pointBGColors_active.push("rgba(0,0,0,1)");
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
            data: data[1]
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
        }]
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
                display: true,
                labels: {
                    color: 'rgb(255, 99, 132)'
                }
            }
        }
          }
        };

        eigPlot = new Chart(canv, config);



        eigPlot.update();

        return canv;
      }


      a_slider.oninput = function(){
        a = this.value/1000.0;
        moto_model.a = a
        document.getElementById("a_sliderval").innerHTML = str(a)
        //update eig data
        eigdata = moto_model.eigStudy(1,10,.1)
        eigPlot.data.datasets[0].data = eigdata[1]
        eigPlot.data.datasets[1].data = eigdata[2]
        eigPlot.update()
        //print("wheel pos udpate: " +str(this.value))
      }
