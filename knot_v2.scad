$fs = 0.1;

step = 1;
width = 15;
tolerance = 0.3;
thickness = 19;
strip_width = 10;
strip_height = 2.5;
brim_height = 4;
brim_width = 2;
bevel = 0.5;
segment_width = 0.01;
strip_length = 2500;
key_pin_shift=1;
diffuser_holder=0.4;

mid_thickness = thickness - 2*(brim_height+strip_height);

echo("Mid thickness: ", mid_thickness);

px = 3;
py = 2;
pz = 3;
cx = 3;
cy = 4;
cz = 2;

function curve(t) = [
    (cx*cos(px*t)+cos(t)),
    (cy*sin(py*t)),
    (cz*sin(pz*t)+sin(t))
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
    -cx*px*sin(px*t)-sin(t),
    cy*py*cos(py*t),
    cz*pz*cos(pz*t)+cos(t)
];

function acceleration(t) = [
    -cx*px*px*cos(px*t) - cos(t),
    -cy*py*py*sin(py*t),
    -cz*pz*pz*sin(pz*t) - sin(t)
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

function exp(i, n) = n > 0 ? exp(i, n-1)*i : 1;


//body(0,180,10);

//body(0,360);

//body(45,135,8);

//covers(0,360,8);

//part(3,36,-1,10,1);
//translate([0,width*2,0])
//part(4,36,-1, 10,1);

//translate([0,-width,0])
//part(6,60,0,8,2);
//translate([0,width,0])
//part(1,60,0);
//placed_part(6,60,8);
//placed_part(7,60,8);
/*
for(i = [1:6]) {
    translate([0,i*100,0])
    part(i,6,0);
}*/

part(1,6,0);
/*
for(i = [1:6]) {
    translate([0,i*width*3,0])
    part(i,6,0);
}*/

module pin(i, size, orientation) {
    translate([orientation*(size/2-0.1),0,(i-2)*3.3])
    rotate([0,orientation*90,0])
    cylinder(h=size, r1=size, r2=0, center=true);
}

function bit_set(i, b) =
    i % exp(2,b) - i % exp(2, b - 1) > 0;

function checksum(i) = 
    (bit_set(i,1) ? 1 : 0) + (bit_set(i,2) ? 1 : 0) + (bit_set(i,3) ? 1 : 0);

//part(5,6,0);
module on_pins(id,size) {
    translate([0,key_pin_shift,0])
    pin(0, size, 1);
    translate([0,-key_pin_shift,0])
    pin(0, size, 1);
    
    for(i = [1:3]) {
        if(bit_set(id,i)) {
            pin(i, size, 1);
        }
    }
    
    if(checksum(id) % 2 == 1) {
        pin(4, size, 1);
    }
}
module off_pins(id,size) {
    for(i = [1:3]) {
        if(!bit_set(id,i)) {
            pin(i, size, -1);
            
        } 
    }
    if(checksum(id) % 2 == 0) {
        pin(4, size, -1);
    }
}

module bevel_cut() {
    translate([0,0,-width/2-brim_width-tolerance])
    rotate([0,45,0])
    cube([1.5*bevel,thickness+2,1.5*bevel], center=true);
    
    translate([0,0,width/2+brim_width+tolerance])
    rotate([0,45,0])
    cube([1.5*bevel,thickness+2,1.5*bevel], center=true);
    
    translate([0,thickness/2,0])
    rotate([0,0,45])
    cube([1.5*bevel,1.5*bevel,width+2*(brim_width+bevel)+2], center=true);
    
    
    translate([0,-thickness/2,0])
    rotate([0,0,45])
    cube([1.5*bevel,1.5*bevel,width+2*(brim_width+bevel)+2], center=true);
    
}

module placed_part(id, count) {
    begin = 360/count * id;
    middle = begin + 180/count;
    end = begin + 360/count;
    
    next_id = id%count + 1;
    
    
    multmatrix(matrix(begin))
    rotate([twist(begin),0,0])
    off_pins(id, 1.5);
    multmatrix(matrix(end))
    rotate([twist(end),0,0])
    on_pins(next_id, 1.5);
    
    difference() {
        body(begin,end);
        
        multmatrix(matrix(begin)) {
            rotate([twist(begin),0,0]) {
                on_pins(id, 1.6);
                bevel_cut();        
            }
        }

        multmatrix(matrix(end)) {
            rotate([twist(end),0,0]) {
                bevel_cut();    
                off_pins(next_id, 1.6);
            }
        }
    }
}

module part(id, count, stick_to) {
    begin = 360/count * id;
    middle = begin + 180/count;
    end = begin + 360/count;
    
    stick_point = middle + stick_to * 180/count;
    
    rotate([0,0,0])
    multmatrix(inverse_matrix(stick_point))
    translate(-f(stick_point))
    placed_part(id,count);
}

module body(begin, end) {
    color("DimGray"){
        difference() {
            extrude(begin,end)     
            cube(size = [segment_width,thickness,width+2*tolerance+2*brim_width], center=true);
            
            
            extrude(begin-step,end+step)     
            translate([0,thickness/2,0])
            cube(size = [segment_width,2*(brim_height+tolerance),width+2*tolerance], center=true);
            
            extrude(begin-step,end+step)     
            translate([0,thickness/2,0])
            cube(size = [segment_width,2*(strip_height+brim_height+tolerance),strip_width+2*tolerance], center=true);
            
            extrude(begin-step,end+step)     
            translate([0,-thickness/2,0])
            cube(size = [segment_width,2*(brim_height+tolerance),width+2*tolerance], center=true);
            
            extrude(begin-step,end+step)     
            translate([0,-(thickness-2*brim_height)/2,0])
            cube(size = [segment_width,2*(strip_height+tolerance),strip_width+2*tolerance], center=true);
            
            multmatrix(matrix(270)) 
            difference() {
                cube([14,10,10], center=true);
                cylinder(h=12,r=3.5, center= true);
            }
        }
        for(i=[0:1]) {
            for(j=[0:1]) {
                extrude(begin,end)
                mirror([0,j,0])
                mirror([0,0,i])
                translate([0,-thickness/2+diffuser_holder,width/2+tolerance])
                rotate([0,90,0])
                cylinder(segment_width,diffuser_holder,diffuser_holder,center=true);
            }
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


//function twist(a) = 0;
//function twist(a) = (1-cos(90*-min(1,max(0,(a-60)/60))))*180;
function twist_phase(a, start, end) = 1-min(1,max(0,(a-start)/(end-start)));
function spline(x) = 3*pow(x,2) - 2*pow(x,3);

function twist(a) = spline(twist_phase(a, 75, 105))*180;

module extrude(begin,end) {
    union() {
        for(a = [begin:step:end-step]) {
            hull() {
                multmatrix(matrix(a))
                rotate([twist(a),0,0])
                children();
                
                multmatrix(matrix(a+step))
                rotate([twist(a+step),0,0])
                children();
            }
        }
    }
}




