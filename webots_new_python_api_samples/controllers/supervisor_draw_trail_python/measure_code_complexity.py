"""This uses the Radon package to compute code readability metrics for two closely matched controllers based
   upon the included Webots sample `supervisor_draw_trail`, one written to use the old Python API and the other
   written to use the new Python API, with the only differences between these being due to their respective API's.
   The controllers are the two "bare_bones" controllers in the same directory as this.
   This presumes that the "current working directory" is at the top level of the "early access" distribution,
   where the readme.md file is, and containing the Webots project folder.  If you run it from another working
   directory, you'll need to change the Path for folder in the first non-import line."""

# /*
#  * Copyright 2022 Justin Fisher.
#  *
#  * Licensed under the Apache License, Version 2.0 (the "License");
#  * you may not use this file except in compliance with the License.
#  * You may obtain a copy of the License at
#  *
#  *     http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing, software
#  * distributed under the License is distributed on an "AS IS" BASIS,
#  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  * See the License for the specific language governing permissions and
#  * limitations under the License.
#  */


import math
from pathlib import Path
import radon, radon.raw, radon.complexity, radon.metrics

# Assuming the current working directory is the one containing the readme.md and the Webots project folder
folder = Path.cwd() / "webots_new_python_api_samples" / "controllers" / "supervisor_draw_trail_python"
new_api_file = folder / "supervisor_draw_trail_new_api_bare_bones.py"
old_api_file = folder / "supervisor_draw_trail_old_api_bare_bones.py"
new_code = new_api_file.read_text()
old_code = old_api_file.read_text()

print("New API Raw Analytics:", radon.raw.analyze(new_code))  # loc=43, lloc=27, sloc=29, comments=19, blank=9, single_comments=5
print("New API Raw Analytics:", radon.raw.analyze(old_code))  # loc=49, lloc=38, sloc=35, comments=19, blank=9, single_comments=5

print("-------------------------------------------------------------------------------")

print("New API Cyclomatic Complexity:", radon.complexity.cc_visit(new_code)[0].complexity)  # complexity = 5 (grade A)
print("Old API Cyclomatic Complexity:", radon.complexity.cc_visit(old_code)[0].complexity)  # complexity = 8 (grade B)

print("-------------------------------------------------------------------------------")

print("New API Halstead Measures:", radon.metrics.h_visit(new_code))  # functions=[('main', HalsteadReport(h1=5, h2=13, N1=14, N2=24, vocabulary=18, length=38, calculated_length=59.715356810271004, volume=158.45715005480787, difficulty=4.615384615384615, effort=731.3406925606516, time=40.63003847559176, bugs=0.05281905001826929))])
print("Old API Halstead Measures:", radon.metrics.h_visit(old_code))  # functions=[('main', HalsteadReport(h1=7, h2=47, N1=35, N2=64, vocabulary=54, length=99, calculated_length=280.71716048325214, volume=569.7338627141835, difficulty=4.76595744680851, effort=2715.3273457016403, time=150.8515192056467, bugs=0.1899112875713945))])

print("-------------------------------------------------------------------------------")

# Inputs to Maintainability Index computations, drawn from above
SLOC = (29, 35)
CC = (5, 8)
Volume = (158, 570)
Comments = (5/43, 5/49)

orig_MI = [(171 - 5.2*math.log(volume) - 0.23*cc - 16.2*math.log(sloc))
          for sloc, cc, volume, comments in zip(SLOC, CC, Volume, Comments)]
print("New API  orig Maintainability Index:", orig_MI[0])  # 88.9743133824789
print("Old API  orig Maintainability Index:", orig_MI[1])  # 78.56605232756279

SEI_MI = [(171 - 5.2*math.log(volume,2) - 0.23*cc - 16.2*math.log(sloc,2) + 50*math.sin(math.sqrt(2.4*comments)))
          for sloc, cc, volume, comments in zip(SLOC, CC, Volume, Comments)]
print("New API   SEI Maintainability Index:", SEI_MI[0])  # 78.37306793228959
print("Old API   SEI Maintainability Index:", SEI_MI[1])  # 62.2064965366009

MVS_MI = [(171 - 5.2*math.log(volume) - 0.23*cc - 16.2*math.log(sloc)) * 100/171
          for sloc, cc, volume, comments in zip(SLOC, CC, Volume, Comments)]
print("New API   MVS Maintainability Index:", MVS_MI[0])  # 52.031762211975966
print("Old API   MVS Maintainability Index:", MVS_MI[1])  # 45.94505984067999

print("New API Radon Maintainability Index:", radon.metrics.mi_visit(new_code, True))  # 81.29991795795722
print("Old API Radon Maintainability Index:", radon.metrics.mi_visit(old_code, True))  # 74.12582099526384

