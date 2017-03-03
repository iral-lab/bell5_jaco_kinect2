sudo yum -y update
sudo yum -y install htop screen numpy git gcc

sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*

sudo python -m pip install --upgrade pip


sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*

pip install --upgrade pip

sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*

pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose pylru

sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*

git clone https://github.com/iral-lab/bell5_jaco_kinect2.git

sudo chown ec2-user:ec2-user -R /usr/lib/python*

cd bell5_jaco_kinect2
mkdir caches

