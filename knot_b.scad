$fs = 0.1;

step = 2;
width = 15;
thickness = 10;
brim = 2;
segment_width = 0.1;
cutter_width = 0.15;
strip_length = 2500;



function curve(t) = [
    2*cos(2*t)-cos(-t),
    2*sin(2*t)-sin(-t),
    sin(3*t)
];

function curve_length(i) = 
    i <= 0 ? 0 : norm(curve(i) - curve(i+1)) + curve_length(i-1);
clen = curve_length(360);
echo("Original curve length:", clen);

function f(t) = curve(t)/clen*strip_length;

function f_length(i) = 
    i <= 0 ? 0 : norm(f(i) - f(i+1)) + f_length(i-1);
echo("Mid point length: ", f_length(360));

function speed(t)  = [
    -4*sin(2*t)-sin(-t),
    4*cos(2*t)+cos(-t),
    3*cos(3*t)
];

function acceleration(t) = [
    -8*cos(2*t)+cos(-t),
    -8*sin(2*t)+sin(-t),
    -9*sin(3*t)
];


function localx(t) = speed(t)/norm(speed(t));
function centripetal(t) = acceleration(t)-(acceleration(t)*localx(t))*localx(t);
function localy(t) = centripetal(t)/norm(centripetal(t));
function localz(t) = cross(localx(t),localy(t));

function top_loc(t) = f(t) + localz(t)*4;
function top_length(i) = 
    i <= 0 ? 0 : norm(top_loc(i) - top_loc(i+1)) + top_length(i-1);
echo("Top strip length: ",top_length(360));

function bottom_loc(t) = f(t) - localz(t)*4;
function bottom_length(i) = 
    i <= 0 ? 0 : norm(
        bottom_loc(i) - bottom_loc(i+1)
    ) + bottom_length(i-1);
echo("Bottom strip length: ",bottom_length(360));

function matrix(t) = [
    [localx(t)[0],localy(t)[0],localz(t)[0],f(t)[0]],
    [localx(t)[1],localy(t)[1],localz(t)[1],f(t)[1]],
    [localx(t)[2],localy(t)[2],localz(t)[2],f(t)[2]],
    [0,0,0,1]
];

function inverse_matrix(t) = [
    concat(localx(t),[0]),
    concat(localy(t),[0]),
    concat(localz(t),[0]),
    [0,0,0,1]
];



//body(0,180);
body(0,360);
covers(0,360);

//part(3,18,0);
//placed_part(3,18);
//placed_part(2,18);

//for(i = [0:1:12]) {
//    translate([0,i*width*2,0])
//    part(i,12,0);
//}

module placed_part(id, count) {
    begin = 360/count * id;
    middle = begin + 180/count;
    end = begin + 360/count;
    hole_location = width/2-3;
    
    difference() {
        body(begin,end);
        multmatrix(matrix(begin+1))
        rotate([90,0,0]) {
            screw_cutter(hole_location);
            screw_cutter(-hole_location);
            
            translate([-2,0,0])
            cube([15,width,3],center=true);
        }
        
    }
    
    multmatrix(matrix(end+1))
    rotate([90,0,0])
    difference() {
        translate([-2,0,0])
        cube([15,width,3],center=true);
        
        screw_cutter(hole_location);
        screw_cutter(-hole_location);
    }
}

module screw_cutter(hole_location) {
    translate([0,hole_location,-thickness/2-1])
    screw_space(thickness+2, 1,40);
    
    translate([0,hole_location,thickness/2-2])
    screw_space(3, 2, 40);
    
    translate([0,hole_location,-1-thickness/2])
    screw_space(3, 2, 6);
}

module part(id, count, stick_to) {
    begin = 360/count * id;
    middle = begin + 180/count;
    end = begin + 360/count;
    
    stick_point = middle + stick_to * 180/count;
    
    rotate([-90,0,0])
    multmatrix(inverse_matrix(stick_point))
    translate(-f(stick_point))
    placed_part(id,count);
}

module screw_space(h, r,fidelity) {
    
    cylinder(h=h, r=r/cos(180/fidelity),$fn=fidelity);
}


module body(begin, end) {
    color("DimGray"){
        difference() {
            extrude(begin,end)     
            cube(size = [segment_width,thickness+brim*2,width+4], center=true);
                    
            extrude(begin-step,end+step)     
            translate([0,(thickness+brim*2+2)/2,0])
            cube(size = [segment_width,brim*2+2,width], center=true);
            
            extrude(begin-step,end+step)     
            translate([0,-(thickness+brim*2+2)/2,0])
            cube(size = [segment_width,brim*2+2,width], center=true);
        }
    }
    
}

module cover(m, begin, end) {
    extrude(begin,end) 
    mirror([0,m,0])
    translate([0,-thickness/2,0])
    arc(width/2,segment_width,8);
}

module covers(begin,end) {
    color("white") {
        cover(0, begin, end);
        cover(1, begin, end);
    }
}



module arc(r,h,fn) {
    rotate(a=[90,0,90]){
        difference(){
            cylinder($fn=fn,h=h,r=r, center=true);
            translate([(r+2)/2,0,0])
            cube([r+2,r*2+2,h+2],center =true);
        }
    }
}

module extrude(begin,end) {
    union() {
        for(a = [begin:step:end-step]) {
            hull() {
                multmatrix(matrix(a))
                children();
                
                multmatrix(matrix(a+step))
                children();
            }
        }
    }
}






