PICTURES="/home/matth/mrcal-java/test_pictures/1280preduced/*"

mrgingham --jobs 8 --gridn 10 $PICTURES > corners.vnl 

# < corners.vnl       \
#   vnl-filter -p x,y | \
#   feedgnuplot --domain --square --set 'xrange [0:1280] noextend' --set 'yrange [720:0] noextend' &

cd mrcal
./mrcal-calibrate-cameras         \
  --corners-cache ../corners.vnl   \
  --lensmodel LENSMODEL_OPENCV8 \
  --focal 1000                  \
  --object-spacing 0.0254       \
  --object-width-n 10           \
  "/home/matth/mrcal-java/test_pictures/1280preduced/*"

# mrcal-show-geometry      \
#   ./camera-0.cameramodel \
#   --show-calobjects      \
#   --unset key            \
#   --set 'xyplane 0'      \
#   --set 'view 80,30,1.5' &

# mrcal-show-residuals    \
#   --histogram           \
#   --set 'xrange [-2:2]' \
#   --unset key           \
#   --binwidth 0.1        \
#   ./camera-0.cameramodel &

# mrcal-show-residuals                   \
#   --magnitudes                         \
#   --set 'cbrange [0:1.5]' ./camera-0.cameramodel &

./mrcal-show-projection-uncertainty ./camera-0.cameramodel --cbmax 10 --unset key &
