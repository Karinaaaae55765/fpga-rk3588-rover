# Build only; no programmer/UART access. Immutable source base.
set root [file normalize [file dirname [info script]]]
set baseline [file join $root source]
set name "camera_udp_task6_candidate_[clock format [clock seconds] -format {%Y%m%d_%H%M%S}]"
set output [file join $root build $name]
if {[file exists $output]} {error "output exists"}
create_project -name $name -dir [file join $root build] -pn GW5AT-LV60PG484AC1/I0 -device_version B
set snap [file join $output source_snapshot];file mkdir $snap
set_option -top_module task6_top
set_option -verilog_std sysv2017
set_option -output_base_name $name
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -timing_driven 1
set_option -correct_hold_violation 1
set_option -place_option 1
set_option -route_option 1
set_option -num_critical_paths 100
foreach src [glob [file join $baseline *]] {
 set tail [file tail $src]
 if {$tail eq "task4_top.sv" || [file isdirectory $src]} {continue}
 if {[file extension $src] in {.sv .v .vh .mem .cst .sdc .json}} {file copy $src [file join $snap $tail]}
}
set route [string map {\\ /} [file join $snap route.mem]]
foreach src [glob [file join $snap *.sv] [file join $snap *.v]] {
 set f [open $src r];fconfigure $f -translation binary -encoding binary;set s [read $f];close $f
 regsub -all {"[^"\n]*route\.mem"} $s "\"$route\"" s
 set f [open $src w];fconfigure $f -translation binary -encoding binary;puts -nonewline $f $s;close $f
 add_file $src
}
add_file [file join $snap board.cst]
add_file [file join $snap timing.sdc]
puts "TASK6_BUILD $output"
run all
set report [file join $output impl pnr ${name}_tr_content.html]
set f [open $report r];set html [read $f];close $f
regsub -all {<[^>]*>} $html { } plain
regsub -all {\s+} $plain { } plain
foreach kind {Setup Hold} {
 if {![regexp "Numbers of $kind Violated Endpoints (\\d+)" $plain match count] || $count!=0} {error "$kind failed"}
}
foreach kind {Recovery Removal} {
 set pos [string first "$kind Paths Table" $plain]
 if {$pos<0 || ![regexp {Data Delay 1 (-?[0-9.]+)} [string range $plain $pos end] match slack] || $slack<0} {error "$kind failed"}
}
set a [string first {Minimum Pulse Width Table:} $plain]
set b [string first {Timing Report By Analysis Type:} $plain $a]
if {$a<0 || $b<=$a || ![regexp -- {-detail 1 (-?[0-9.]+)} [string range $plain $a $b] match slack] || $slack<0} {error "Pulse width failed"}
puts "INTERNAL_TIMING_PASS $output ; not programmed, external timing not signed off"
