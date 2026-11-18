"""Exploratory linear compliance decomposition, not a structural certification."""
import math

def screen(force_n,elastic_mpa=69000,shear_mpa=26000,revision='wide'):
 vals=[force_n,elastic_mpa,shear_mpa]
 if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in vals):raise ValueError('invalid scalar')
 if force_n<0 or elastic_mpa<=0 or shear_mpa<=0 or revision not in ['slender','wide']:raise ValueError('domain')
 # CAD-derived section sizes with a declared beam-centerline idealization. Joint zones and frame compliance are omitted.
 side,stem_x,beam_height=(6,73,3) if revision=='slender' else (16,70,8)
 arm_centroid_z=199+beam_height/2
 height=arm_centroid_z-116;load_drop=arm_centroid_z-185;arm=146-stem_x;beam_length=117.6-stem_x;overhang=146-117.6
 i_stem=side**4/12;j_stem=.1406*side**4;i_beam=beam_height*side**3/12
 bending=(height**3/3-load_drop*height**2+load_drop**2*height)/(elastic_mpa*i_stem)
 torsion=arm**2*height/(shear_mpa*j_stem)
 beam=(beam_length**3/3+overhang*beam_length**2+overhang**2*beam_length)/(elastic_mpa*i_beam)
 delta=force_n*(bending+torsion+beam);energy=.5*force_n*delta/1000
 if not all(math.isfinite(v) for v in [delta,energy,bending,torsion,beam]):raise ValueError('derived nonfinite')
 return {'revision':revision,'armCentroidZmm':arm_centroid_z,'verticalCenterlineLengthMm':height,'loadBelowArmCentroidMm':load_drop,'attachmentFaceCentroidXmm':117.6,'forceN':force_n,'elasticModulusHypothesisMPa':elastic_mpa,'shearModulusHypothesisMPa':shear_mpa,'stemBendingMm':force_n*bending,'stemTorsionMm':force_n*torsion,'armBendingMm':force_n*beam,'totalEstimatedMm':delta,'elasticEnergyJ':energy,'accepted':False}
