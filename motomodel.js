function MotorcycleModel(dbg,lam,a,xp,zp,ut,ufx,Lf,hrf,mrf,mff,Rfw,mfw,Rrw,mrw,bsteer){
  this.xp = xp //x distance from rear axle to top of triple clamp pivot
  this.zp = zp //height of top of triple clamp pivot
  this.ut = ut //triple clamp offset
  this.ufx = ufx //fork/axle offset along x direction
  this.Lf = Lf //length of forks at static from top of clamp to axle
  this.dbg= dbg
  this.g = 9.81
  //angle from ground to steer axis
  this.lam = lam
  //dist from rear axle to CG (x)
  this.a = a
  //rear CG height
  this.hrf = hrf
  //rear frame mass
  this.mrf = mrf
  //front fork mass
  this.mff = mff
  //front wheel radius
  this.Rfw = Rfw
  //front wheel mass
  this.mfw = mfw
  //rear wheel radius
  this.Rrw = Rrw
  //rear wheel mass
  this.mrw = mrw
  this.bsteer = bsteer



  this.buildMDK = function(v){
    //compute wheelbase
    this.xfw = this.xp + this.ut/Math.cos(this.lam) + this.Lf*Math.cos(this.lam) + this.ufx
    // console.log("computed wheelbase: ",this.xfw)
    this.b = this.xfw
    // console.log(this.b)
    this.zff = this.zp+this.ut/Math.sin(this.lam) - 0.5*this.Lf*Math.sin(this.lam)//height of front frame mass
    this.xff = this.xp + 0.5*this.Lf*Math.cos(this.lam)+this.ut/Math.cos(this.lam)//x position of front frame mass
    // console.log("computed xff: ",this.xff)
    // console.log("computed zff: ",this.zff)
    this.c = (this.xp+this.zp/Math.tan(this.lam))-this.xfw
    // console.log("computed trail: ",this.c)
    //gyroscopic inertias
    this.Jyyf = 0.9*this.mfw*Math.pow(this.Rfw,2)
    this.Jyyr = 0.9*this.mrw*Math.pow(this.Rrw,2)
    //rear frame height
    this.mr = this.mrf+this.mrw
    this.h = (this.mrf*this.hrf+this.mrw*Rrw)/(this.mr)
    //front frame totals
    this.mf = this.mff+this.mfw
    this.xf = (this.mff*this.xff+this.mfw*this.xfw)/(this.mf)
    this.hf = (this.zff*this.mff+this.Rfw*this.mfw)/(this.mf)
    //perp dist between steer axis and front frame
    this.u = this.hf*Math.cos(this.lam)-(this.b+this.c-this.xf)*Math.sin(this.lam)
    //this.update(v)
    //gyroscopic moment terms
    Sf = this.Jyyf/this.Rfw
    Sr = this.Jyyr/this.Rrw
    St = Sf+Sr

    M11 = this.mr*Math.pow(this.h,2)+this.mf*Math.pow(this.hf,2)
    M12 = -this.mf*this.hf*this.u - this.c*Math.sin(this.lam)/this.b*(this.mr*this.a*this.h+this.mf*this.xf*this.hf)
    M21 = M12
    M22 = this.mf*Math.pow(this.u,2)+2*this.mf*this.xf*this.u*this.c*Math.sin(this.lam)/this.b + Math.pow(this.c,2)*Math.pow(Math.sin(this.lam),2)/Math.pow(this.b,2)*(this.mr*Math.pow(this.a,2)+this.mf*Math.pow(this.xf,2))

    D11 = 0
    D12 = -v*Math.sin(this.lam)/this.b*(this.mr*this.h*this.a+this.mf*this.xf*this.hf)-St*v*this.c*Math.sin(lam)/this.b-Sr*v*Math.sin(lam)
    D21 = (St*v*this.c*Math.sin(this.lam)/this.b + Sf*v*Math.sin(this.lam))
    D22 = v*Math.sin(this.lam)/this.b*(this.mf*this.xf*this.u + this.c*Math.sin(this.lam)/this.b*((this.mr*Math.pow(a,2) + this.mf*Math.pow(this.xf,2))+this.mr*this.a*this.c+this.mf*this.xf*this.c)) + this.mf*this.u*v*this.c*Math.sin(this.lam)/this.b + this.bsteer

    K11 = -this.g*(this.mr*this.h+this.mf*this.u)
    K12 = -Math.pow(v,2)*Math.sin(this.lam)/this.b*( this.mr*this.h+this.mf*this.hf)+(this.mf*this.xf+this.mr*this.a)*this.g*this.c*Math.sin(this.lam)/this.b+this.mf*this.g*this.u - Math.pow(v,2)*St*Math.sin(this.lam)/this.b
    K21 = this.c*(this.mr*this.a+this.mf*this.xf)*this.g*Math.sin(this.lam)/this.b + this.mf*this.g*this.u
    K22 = -(this.mr*this.a+this.mf*this.xf)*this.c*this.g*Math.sin(this.lam)*Math.cos(this.lam)/this.b - this.mf*this.g*this.u + this.c*Math.pow(Math.sin(lam),2)*Math.pow(v,2)/Math.pow(this.b,2)*(this.mf*this.xf+this.mr*this.a) +this.mf*this.u*Math.pow(v,2)*Math.sin(this.lam)/this.b + Sf*Math.pow(v,2)*Math.cos(this.lam)*Math.sin(this.lam)/this.b
    // MDK form mass matrix:
    this.M = nj.array([[M11,M12],[M21,M22]])
    // MDK form damping matrix
    this.D = nj.array([[D11,D12],[D21,D22]])
    // MDK form stiffness matrix
    this.K = nj.array([[K11,K12],[K21,K22]])
  }

  this.buildSS4 = function(v){
    //update the model in MDK form first
    this.buildMDK(v)
    //states are [roll steer, rollrate, steerrate].T
    //now assemble the appropriate matrices
    //for A, first stack zeros with identity
    Atop = nj.concatenate(nj.zeros([2,2]),nj.identity(2))
    //now do the inversions for derivatives
    negMinv = nj.subtract(nj.zeros([2,2]),inv22(this.M))

    MinvK = nj.dot(negMinv,this.K)
    MinvD = nj.dot(negMinv,this.D)
    Abottom = nj.concatenate(MinvK,MinvD)
    //construct A matrix by using horizontal concatenation of transposes,then taking transpose
    this.A4 = nj.concatenate(Atop.T,Abottom.T).T
    //construct B matrix
    Btop = nj.zeros([2,2])
    Bbottom = nj.dot(inv22(this.M),nj.identity(2))
    this.B4 = nj.concatenate(Btop.T,Bbottom.T).T
    this.getEigs4() //update so we have access to the eigs, and update stability boolean

  }

  this.getSensitivity = function(){
    if(this.isStable){
      sens_array = nj.dot(inv22(this.K),nj.array([[0],[1]]))
      // console.log(sens_array)
      this.sensitivity_steer = (1.0/(180.0/3.1415*Math.abs(sens_array.selection.data[1]))).toFixed(3)

    }
    else{
      this.sensitivity_steer = "N/A (not stable)"
    }
    // console.log(this.sensitivity_steer)
  }

  this.buildRoadSS4 = function(){

  }

  this.checkStable = function(){

    if(this.eigs_re == null) {
        this.isStable= false;
        return
     }
      for(var i = 0; i < this.eigs_re.length; i++) {
          if(this.eigs_re[i] >= 0){
             this.isStable = false;
             break
           }
           else{
             this.isStable = true;
           }
      }
  }

  this.getEigs4 = function(){
    var A4math = math.matrix([ [this.A4.get(0,0),this.A4.get(0,1),this.A4.get(0,2),this.A4.get(0,3)],[this.A4.get(1,0),this.A4.get(1,1),this.A4.get(1,2),this.A4.get(1,3)],[this.A4.get(2,0),this.A4.get(2,1),this.A4.get(2,2),this.A4.get(2,3)],[this.A4.get(3,0),this.A4.get(3,1),this.A4.get(3,2),this.A4.get(3,3)]])
    //console.log(A4math)
    try{
      eigs = math.eigs(A4math,.1)
      this.eigs = eigs
      this.eigs_re = [math.re(eigs.values._data[0]),math.re(eigs.values._data[1]),math.re(eigs.values._data[2]),math.re(eigs.values._data[3])]
      this.eigs_im = [math.im(eigs.values._data[0]),math.im(eigs.values._data[1]),math.im(eigs.values._data[2]),math.im(eigs.values._data[3])]
      this.checkStable()
      return [this.eigs_re,this.eigs_im,this.isStable]
    }
    catch(err){
      eigs = err
      // console.log(err.values)
      return [[NaN,NaN,NaN,NaN],[NaN,NaN,NaN,NaN],false]
    }

  }

  this.stepResponse = function(v,stepMag,stepTime,dt){
    var tnow = 0
    var rollvec = []
    var steervec = []
    this.buildSS4(v)
    this.getSensitivity()
    //we are going to use Euler integration.
    // x(k) = (eye(4)+A*dt)*x(k-1)+B*dt*u(k-1)
    var Ad = nj.add(nj.identity(4),this.A4.multiply(dt))
    var Bd = this.B4.multiply(dt)
    this.Ad = Ad
    this.Bd = Bd
    var u = nj.array([[0],[stepMag]])
    var xd = nj.zeros([4,1])

    //set up loop
    while (tnow<=stepTime){
      //add to my vector
      rollvec.push({x:tnow,y:xd.get(0,0)*180/3.1415})
      steervec.push({x:tnow,y:xd.get(1,0)*180/3.1415})
      //update the simulation ONLY if the vehicle hasn't tipped over.
      if(Math.abs(xd.get(0,0))<1.57){
        xd = nj.add(nj.dot(Ad,xd),nj.dot(Bd,u))
      }
      else{
        xd = nj.array([[1.57*Math.sign(xd.get(0,0))],[xd.get(1,0)],[0],[0]])
      }
      //increment time
      tnow+=dt
      // console.log(rollvec)
    }
    return [rollvec,steervec]

  }

  this.eigStudy = function(vmin,vmax,inc){
    var vvec = []
    var revec = []
    var imvec = []
    var stabilityvec = []
    vnow = vmin
    while (vnow<=vmax){
      // console.log(vnow)
      //build the model at this speed
      this.buildSS4(vnow)
      //get the eigs
      eigs = this.getEigs4()
      // console.log(eigs)
      vvec.push(vnow/0.447)
      revec.push({x: vnow/0.447,y: eigs[0][0]},
                {x: vnow/0.447,y: eigs[0][1]},
                {x: vnow/0.447,y: eigs[0][2]},
                {x: vnow/0.447,y: eigs[0][3]})

      imvec.push({x: vnow/0.447,y: eigs[1][0]},
                {x: vnow/0.447,y: eigs[1][1]},
                {x: vnow/0.447,y: eigs[1][2]},
                {x: vnow/0.447,y: eigs[1][3]})
      stabilityvec.push(this.isStable)
      stabilityvec.push(this.isStable)
      stabilityvec.push(this.isStable)
      stabilityvec.push(this.isStable)

      vnow+=inc
    }
    return [vvec,revec,imvec,stabilityvec]
  //now set the
  }

  this.updateModel = function(v){
    this.buildSS4(v)
    // this.getEigs4()
    this.eigdata = this.eigStudy(1,15,.1)
    if(this.dbg){
      console.log("A matrix: \n"+this.A4.toString())
      console.log("B matrix: \n"+this.B4.toString())
      console.log("eigs_re: "+this.eigs_re)
      console.log("eigs_im: "+this.eigs_im)
    }

  }

}

function det22(M){
  return M.get(0,0)*M.get(1,1)-M.get(1,0)*M.get(0,1)
}

function inv22(M){
  //compute the inverse of a 2x2 matrix
  det = det22(M)
  return nj.array([[M.get(1,1)/det,-M.get(0,1)/det],[-M.get(1,0)/det,M.get(0,0)/det]])
}



// //////// TEST ///////
// lam = 1.13
// hrf = .25606
// a = .3386//meters, distance from rear axle to CG in x direction
// b = .767//meters, wheelbase of bike
// c = .023//.08//meters, trail
// hrf = .25606//meters, rear frame CG height
// mr = 11.065//kg, rear frame mass inc. rider
// xff = .62218//position of front frame CG
// yff = 0
// zff = .46531
// mff = 2.2047 //kg, fork mass
// Rfw = .15875
// mfw = 1.486 //kg, wheel mass
// Rrw = 0.15875 // radius of real wheel
// mrw = 2.462 //mass of rear wheel
// v = 4 //m/s, fwd speed


// var moto_model = new MotorcycleModel(true,lam,a,b,c,hrf,mr,xff,yff,zff,mff,Rfw,mfw,Rrw,mrw)
// moto_model.updateModel(v)
