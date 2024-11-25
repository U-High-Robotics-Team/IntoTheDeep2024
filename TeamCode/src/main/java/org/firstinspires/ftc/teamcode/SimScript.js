// global simulation and graphics paremeters
const WIDTH = 3.72; // meters: approx FTC field size
const HEIGHT = 3.72; // meters
const TIMESTEP = 0.05; // seconds
// TODO read user screen info and set default canvas size to match
let viewScale = 600 / WIDTH; // pixels per meter

var field = createField();
var bot = null;
reset();
animate();

function reset() {
  field.clear()
  field.drawGridlines();
  field.drawAxes();
  bot = createBotV1();
  bot.place(field);
}

function animate(){
  field.clear();
  field.drawGridlines();
  field.drawAxes();
  bot.update();
  // TODO have animation rate match real world
  window.requestAnimationFrame(animate);
}

function move() {
  let x = document.getElementById("x").value / 1;  // the /1 fake math appears to be necessary to convert to a number
  let y = document.getElementById("y").value / 1;
  let rot = (document.getElementById("rot").value * 3.14159) / 180; // deg to radians
  bot.move(x, y, rot);
}

function moveTo() {
  let x = document.getElementById("x").value / 1;  // the /1 fake math appears to be necessary to convert to a number
  let y = document.getElementById("y").value / 1;
  let rot = (document.getElementById("rot").value * 3.14159) / 180; // deg to radians
  bot.moveTo(x, y, rot);
}


function createField() {
  let object = {
    canvas: document.getElementById("canvas"),
    ctx: this.canvas.getContext("2d"),
    getContext: function () {
      return this.ctx;
    },
    drawGridlines: function () {
      this.ctx.strokeStyle = "lightgray";
      let gridSize = 0.6172;    // FTC tile size
      // Draw vertical lines
      this.ctx.lineWidth = 1 / viewScale;
      for (let x = -3 * gridSize; x <= 3 * gridSize; x += gridSize) {
        this.ctx.beginPath();
        this.ctx.moveTo(x, - HEIGHT / 2);
        this.ctx.lineTo(x, HEIGHT / 2);
        this.ctx.stroke();
      }
      // Draw horizontal lines
      for (let y = -3 * gridSize; y <= 3 * gridSize; y += gridSize) {
        this.ctx.beginPath();
        this.ctx.moveTo(-WIDTH / 2, y);
        this.ctx.lineTo(WIDTH / 2, y);
        this.ctx.stroke();
      }
    },
    drawAxes: function () {
      this.ctx.lineWidth = 1 / viewScale;
      this.ctx.strokeStyle = "black";
      this.ctx.beginPath();
      this.ctx.moveTo(-WIDTH / 2, 0);
      this.ctx.lineTo(WIDTH / 2, 0);
      this.ctx.stroke();
      this.ctx.beginPath();
      this.ctx.moveTo(0, -HEIGHT / 2);
      this.ctx.lineTo(0, HEIGHT / 2);
      this.ctx.stroke();
      this.ctx.strokeStyle = "red";
      this.ctx.beginPath();
      this.ctx.moveTo(-WIDTH / 2, -HEIGHT / 2);
      this.ctx.lineTo(WIDTH / 2, -HEIGHT / 2);
      this.ctx.lineWidth = 3 / viewScale;
      this.ctx.stroke();
    },
    clear: function () {
      this.ctx.clearRect(-WIDTH/2, HEIGHT/2, WIDTH, -HEIGHT);
    },
  };
  // TODO zoom and pan
  object.ctx.scale(viewScale, -viewScale);                // scale meters to pixels and invert y-axis
  object.ctx.translate(WIDTH / 2, -HEIGHT / 2);   // move 0,0 to middle of screen
  return object;
}

function createBotV1() {
  let object = {
    // world position vars
    xPos: 0,
    yPos: 0,
    theta: 0, // angle in radians CW from world +x axis
    xTarget: 0,
    yTarget: 0,
    thetaTarget: 0,

    // kinematics vars
    angVelFL:  0,  // radians / sec
    angVelFR:  0,
    angVelBL:  0,
    angVelBR:  0,
    wheelRadius: 0.05,  // meters
    axleLength: 0.338,  // meters
    isMoving: false,
    positionTolerance: 0.05,  // meters
    rotationTolerance: 0.1,  // radians

    // screen graphics vars
    ctx: null,
    xSize: .435, // bot LENGTH (x) in meters
    ySize: .45, // both WIDTH (y) in meters
    stoppedColor: "#f00",
    movingColor: "#faa",
    place: function (sc) {
      this.ctx = sc.getContext();
    },
    draw: function () {
      // drawing a shape with translation and rotation is done by transforming the coordinate system
      // so: first, save the original coordinate system, then transform and draw
      // finally, restore the original coordinate system
      this.ctx.save();
      this.ctx.translate(this.xPos, this.yPos);
      this.ctx.rotate(-this.theta);
      this.ctx.lineWidth = 1 / viewScale;
      if (this.isMoving) this.ctx.strokeStyle = this.movingColor;
      else this.ctx.strokeStyle = this.stoppedColor;
      this.ctx.fillStyle = this.movingColor;
      this.ctx.strokeRect(
        -this.xSize / 2,
        -this.ySize / 2,
        this.xSize,
        this.ySize
      );
      this.ctx.beginPath();
      this.ctx.moveTo(this.xSize / 2, 0);
      this.ctx.lineTo(this.xSize / 4, -this.ySize / 4);
      this.ctx.lineTo(this.xSize / 4, this.ySize / 4);
      this.ctx.lineTo(this.xSize / 2, 0);
      this.ctx.fill();
      this.ctx.restore();
    },
    moveTo: function (x, y, t) {
      // move to an absolute world position by updating targets
      // if moving, this will replace the old target with a new one
      this.xTarget = x;
      this.yTarget = y;
      this.thetaTarget = t;
    },
    move: function (dX, dY, dT) {
      // move an amount relative to the current postion
      // if moving, this will replace the old target with a new one
      this.xTarget = this.xPos + dX;
      this.yTarget = this.yPos + dY;
      this.thetaTarget = this.theta + dT;
    },
    stop: function () {
      this.move(0, 0, 0, 1);
    },
    update: function () {
      this.doPID();   // calculates the desired wheel speeds - this part is the physical robot controller
      this.doSim();  // calculates how the virtual (on-screen) robot should move to mimic the physical robot
      this.draw();
    },
    doPID: function () {
      let positionError = Math.hypot(this.xTarget - this.xPos, this.yTarget - this.yPos);
      let rotationError = this.thetaTarget - this.theta;

      // TODO implement a user-defined travel time
      let travelTime = 3
      let xVel = (this.xTarget - this.xPos) / travelTime;
      let yVel = (this.yTarget - this.yPos) / travelTime;
      let tVel = (this.thetaTarget - this.theta) / travelTime;
      //console.log(xVel+","+yVel+","+tVel);
      this.angVelFL = (xVel - yVel - this.axleLength * tVel) / this.wheelRadius; // + - -
      this.angVelFR = (xVel + yVel + this.axleLength * tVel) / this.wheelRadius; // + + +
      this.angVelBL = (xVel + yVel - this.axleLength * tVel) / this.wheelRadius; // + + -
      this.angVelBR = (xVel - yVel + this.axleLength * tVel) / this.wheelRadius; // + - +
      //console.log(this.angVelFL+","+this.angVelFR+","+this.angVelBL+","+this.angVelBR);
    },
    doSim: function(){
      let xVel = (this.angVelFL + this.angVelFR + this.angVelBL + this.angVelBR) * this.wheelRadius / 4;
      let yVel = (-this.angVelFL + this.angVelFR + this.angVelBL - this.angVelBR) * this.wheelRadius / 4;
      let tVel = (-this.angVelFL + this.angVelFR - this.angVelBL + this.angVelBR) * this.wheelRadius / 4 / this.axleLength;
      //console.log(xVel+","+yVel+","+tVel);
      this.xPos += xVel * TIMESTEP;
      this.yPos += yVel * TIMESTEP;
      this.theta += tVel * TIMESTEP;
    },
  };
  return object;
}
