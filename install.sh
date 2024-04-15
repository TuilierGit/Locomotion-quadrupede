wget https://rhoban.github.io/quadruped/quadruped.zip
unzip quadruped.zip
python -m venv profil_robotique
rm quadruped/control.py
mv quadruped/* .
rm -r quadruped
rm quadruped.zip
# source profil_robotique/bin/activate
# pip3 install numpy scipy pybullet matplotlib
