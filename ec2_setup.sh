sudo yum -y update
sudo yum -y install htop screen numpy git gcc

sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib64/python*

sudo python -m pip install --upgrade pip


sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib64/python*

pip install --upgrade pip

sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib64/python*

pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose pylru
pip install -U scikit-learn

sudo chown ec2-user:ec2-user -R /usr/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib/python*
sudo chown ec2-user:ec2-user -R /usr/local/lib64/python*

git clone https://github.com/iral-lab/bell5_jaco_kinect2.git

sudo chown ec2-user:ec2-user -R /usr/lib/python*

cd bell5_jaco_kinect2
touch ~/bell5_jaco_kinect2/.on_aws
mkdir caches
mkdir clouds

