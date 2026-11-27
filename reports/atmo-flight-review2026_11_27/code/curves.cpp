// Review evidence: sweep the REAL drag.h laws and print CSVs.
// Build: g++ -std=c++20 -O2 -I../../src -I../../middleware/glm curves.cpp -o curves
#include <cstdio>
#include <cmath>
#include "drag.h"

int main() {
    // 1. lift slope vs Mach (cl 6.0, the shipped wing slope)
    {
        FILE *f = fopen("lift_slope.csv", "w");
        fprintf(f, "mach,slope,pg,ackeret\n");
        for(double m = 0.0; m <= 3.0001; m += 0.01) {
            const double pg = (m < 1.0) ? 6.0 / std::sqrt(1 - m * m) : 0.0;
            const double ak = (m > 1.0) ? 4.0 / std::sqrt(m * m - 1) : 0.0;
            fprintf(f, "%.3f,%.6f,%.6f,%.6f\n", m, liftSlopeMach(6.0, m), pg, ak);
        }
        fclose(f);
    }
    // 2. wave drag vs Mach for the shipped wing thickness band
    {
        FILE *f = fopen("wave_cd.csv", "w");
        fprintf(f, "mach,tc044,tc060,tc086\n");
        for(double m = 0.0; m <= 3.0001; m += 0.01) {
            fprintf(f, "%.3f,%.6f,%.6f,%.6f\n", m,
                    waveCd(0.044, m), waveCd(0.060, m), waveCd(0.086, m));
        }
        fclose(f);
    }
    // 3. lift curve / soft stall (cl 6, stall 0.35 rad = 20.05 deg)
    {
        FILE *f = fopen("lift_curve.csv", "w");
        fprintf(f, "alpha_deg,cl_at_m0,cl_at_m075,cl_at_m15\n");
        const double s0 = liftSlopeMach(6.0, 0.0);
        const double s075 = liftSlopeMach(6.0, 0.75);
        const double s15 = liftSlopeMach(6.0, 1.5);
        for(double a = -5.0; a <= 50.001; a += 0.25) {
            const double r = a * std::numbers::pi / 180.0;
            fprintf(f, "%.2f,%.6f,%.6f,%.6f\n", a,
                    liftCurve(s0, r, 0.35), liftCurve(s075, r, 0.35),
                    liftCurve(s15, r, 0.35));
        }
        fclose(f);
    }
    // 4. jet thrust vs airspeed at three densities, stock 2 m jet and the
    //    #1206 demo re-declaration (fan 3000, 3 kg/s, v_e 3200, A 0.35).
    {
        FILE *f = fopen("jet_thrust.csv", "w");
        fprintf(f, "v,rho,stock_N,demo_N,stock_exit,demo_exit\n");
        // Kerbin datum, 10 km and 20 km (profiles.csv, quadrature of the shipped table)
        const double rhos[] = {1.225, 0.4165, 0.0948};
        for(double rho : rhos) {
            for(double v = 0.0; v <= 1600.01; v += 5.0) {
                const double air = rho * 0.35 * v;
                const double d = jetAirGate(rho, dryAirOxidizerShare());
                const double es = jetRamExitSpeed(v, air, 1.2 * d, 550.0);
                const double ed = jetRamExitSpeed(v, rho * 0.35 * v, 3.0 * d, 3200.0);
                fprintf(f, "%.1f,%.4f,%.3f,%.3f,%.3f,%.3f\n", v, rho,
                        jetThrust(v, rho, dryAirOxidizerShare(), 90000, 1.2, 550, 0.35),
                        jetThrust(v, rho, dryAirOxidizerShare(), 3000, 3.0, 3200, 0.35),
                        es, ed);
            }
        }
        fclose(f);
    }
    // 5. jet gate vs density for the shipped mixes (dry air, Eve CO2 trace,
    //    Jool H2/He) -- the composition story.
    {
        FILE *f = fopen("jet_gate.csv", "w");
        fprintf(f, "rho,dry,eve_trace,jool\n");
        for(double rho = 0.0; rho <= 6.0001; rho += 0.02) {
            fprintf(f, "%.3f,%.6f,%.6f,%.6f\n", rho,
                    jetAirGate(rho, dryAirOxidizerShare()),
                    jetAirGate(rho, 0.000959), jetAirGate(rho, 0.0));
        }
        fclose(f);
    }
    printf("curves written\n");
    return 0;
}
