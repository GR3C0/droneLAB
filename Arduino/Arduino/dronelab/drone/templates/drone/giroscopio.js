window.addEventListener("deviceorientation",function(event) {
  alpha = Math.round(event.alpha);
  beta = Math.round(event.beta);
  gamma = Math.round(event.gamma);
}, true);
alert("Hola Mundo2");
