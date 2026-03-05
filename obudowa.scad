// Szambo TOF Sensor - Obudowa (Enclosure)
// 2-częściowa obudowa do druku 3D z miejscem na uszczelkę i otworem na czujnik

$fn = 60; // Rozdzielność łuków

// --- Parametry Wymiarów ---
inner_w = 40;     // Szerokość wewnętrzna
inner_l = 60;     // Długość wewnętrzna
inner_h = 25;     // Wysokość wewnętrzna ciała (bez pokrywki)
wall_t = 3;       // Grubość ścianek

post_r = 4;       // Promień słupka montażowego w rogach
screw_d = 2.5;    // Średnica otworu pod gwint np. wkręt do plastiku M3
screw_head_d = 5.5; // Średnica główki śruby
screw_head_h = 2; // Wysokość zagłębienia główki

groove_w = 2;     // Szerokość rowka pod uszczelkę (gumę/silikon)
groove_d = 1.5;   // Głębokość rowka

tof_hole_w = 12;  // Szerokość otworu na czujnik VL53L0X
tof_hole_h = 6;   // Wysokość otworu na czujnik

// --- Obliczenia ---
outer_w = inner_w + 2*wall_t;
outer_l = inner_l + 2*wall_t;
outer_h = inner_h + wall_t; // Ścianka tylko na dnie
lid_t = 3;                  // Grubość samej pokrywki

// Odległości środków śrub 
screw_x = outer_w - post_r + 0.5;
screw_y = outer_l - post_r + 0.5;
screw_x1 = post_r - 0.5;
screw_y1 = post_r - 0.5;


// --- MAIN ---
// Odkomentuj tylko jedną z części żeby wyrenderować przed eksportem STL
translate([0, 0, 0]) base();
translate([outer_w + 10, 0, 0]) lid();

// --- MODUŁY ---

module base() {
    difference() {
        union() {
            // Główna bryła
            cube([outer_w, outer_l, outer_h]);
        }
        
        // Wnętrze (wydrążenie)
        translate([wall_t, wall_t, wall_t])
            cube([inner_w, inner_l, inner_h + 1]);
        
        // Wycięcie na rowek uszczelki (na samej górze krawędzi)
        translate([wall_t/2 + groove_w/2, wall_t/2 + groove_w/2, outer_h - groove_d]) {
            difference() {
                cube([outer_w - wall_t - groove_w, outer_l - wall_t - groove_w, groove_d + 1]);
                translate([groove_w, groove_w, -1])
                    cube([outer_w - wall_t - 3*groove_w, outer_l - wall_t - 3*groove_w, groove_d + 3]);
            }
        }
        
        // Otwór na czujnik TOF
        // Otwór umieszczamy na prawym boku obudowy (X = outer_w)
        translate([outer_w - wall_t - 1, outer_l/2, outer_h/2]) {
             // Owalny/prostokątny otwór
             cube([wall_t + 2, tof_hole_w, tof_hole_h], center=true);
        }
        
        // Otwory na śruby (na dole zostawiamy pełny plastik dla słupków)
        screw_holes(screw_d, outer_h - 10, offset_z = 10);
    }
    
    // Dodajemy słupki wewnętrzne żeby śruby miały się czego trzymać
    difference() {
        union() {
            translate([screw_x1, screw_y1, wall_t]) cylinder(r=post_r, h=inner_h);
            translate([screw_x1, screw_y, wall_t]) cylinder(r=post_r, h=inner_h);
            translate([screw_x, screw_y1, wall_t]) cylinder(r=post_r, h=inner_h);
            translate([screw_x, screw_y, wall_t]) cylinder(r=post_r, h=inner_h);
        }
        screw_holes(screw_d, outer_h + 1, offset_z = 2);
    }
}


module lid() {
    difference() {
        cube([outer_w, outer_l, lid_t]);
        // Otwory przelotowe w lince z pogłębieniem
        lid_screw_holes();
    }
    
    // Pokrywka pozostaje płaska od spodu - dociśnie gumę leżącą w rowku części głównej.
}

// Otwory gwintowane w bazie
module screw_holes(d, h, offset_z=0) {
    translate([screw_x1, screw_y1, offset_z]) cylinder(d=d, h=h);
    translate([screw_x1, screw_y, offset_z]) cylinder(d=d, h=h);
    translate([screw_x, screw_y1, offset_z]) cylinder(d=d, h=h);
    translate([screw_x, screw_y, offset_z]) cylinder(d=d, h=h);
}

// Otwory w pokrywie
module lid_screw_holes() {
    hole_radius = screw_d/2 + 0.3; // odrobine luźniejsze (przelotowe)
    
    module single_hole() {
        cylinder(r=hole_radius, h=lid_t + 2, center=true);
        // główka
        translate([0, 0, lid_t - screw_head_h + 0.01])
            cylinder(d=screw_head_d, h=screw_head_h + 1);
    }
    
    translate([screw_x1, screw_y1, 0]) single_hole();
    translate([screw_x1, screw_y, 0]) single_hole();
    translate([screw_x, screw_y1, 0]) single_hole();
    translate([screw_x, screw_y, 0]) single_hole();
}
