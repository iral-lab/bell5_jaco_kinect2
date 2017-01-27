sudo yum -y update
sudo yum -y install htop screen numpy git gcc

sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo python -m pip install --upgrade pip


sudo chown ec2-user:ec2-user -R /usr/lib/python*

pip install --upgrade pip

sudo chown ec2-user:ec2-user -R /usr/lib/python*
pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose

sudo chown ec2-user:ec2-user -R /usr/lib/python*

git clone https://github.com/iral-lab/bell5_jaco_kinect2.git

cd bell5_jaco_kinect2
mkdir caches

