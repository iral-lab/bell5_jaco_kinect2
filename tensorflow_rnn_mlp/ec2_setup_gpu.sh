# http://mortada.net/tips-for-running-tensorflow-with-gpu-support-on-aws.html
# ubuntu/images/hvm-ssd/ubuntu-trusty-14.04-amd64-server-20170405 (ami-772aa961)

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y awscli build-essential python-pip python-dev git python-numpy swig python-dev default-jdk zip zlib1g-dev htop screen git gcc ruby

wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_8.0.61-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu1404_8.0.61-1_amd64.deb
sudo apt-get update
sudo apt-get install cuda -y

sudo reboot

# scp -i ~/.ssh/neilrbell_solo_20160113.pem Downloads/cudnn-8.0-linux-x64-v6.0.tgz ubuntu@34.229.153.57:~/
tar zxf cudnn-8.0-linux-x64-v6.0.tgz
cd cuda
sudo cp lib64/* /usr/local/cuda/lib64/
sudo cp include/* /usr/local/cuda/include/
cd ~


echo "export CUDA_HOME=/usr/local/cuda" >> ~/.bashrc
echo "export CUDA_ROOT=/usr/local/cuda" >> ~/.bashrc
echo "export PATH=$PATH:/usr/local/cuda/bin" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64" >> .bashrc

export CUDA_HOME=/usr/local/cuda
export CUDA_ROOT=/usr/local/cuda
export PATH=$PATH:/usr/local/cuda/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64

sudo reboot

time nvidia-smi
sudo nvidia-smi -pm 1
sudo nvidia-smi --auto-boost-default=0
sudo nvidia-smi -ac 2505,875
time nvidia-smi




sudo chown ubuntu:ubuntu -R /usr/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib64/python*

sudo python -m pip install --upgrade pip

sudo chown ubuntu:ubuntu -R /usr/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib64/python*

pip install --upgrade pip

sudo chown ubuntu:ubuntu -R /usr/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib64/python*

pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose pylru
pip install -U scikit-learn

sudo pip install tensorflow-gpu

sudo chown ubuntu:ubuntu -R /usr/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib/python*
sudo chown ubuntu:ubuntu -R /usr/local/lib64/python*

python -c "import tensorflow as tf; print tf.__version__"

git clone https://github.com/iral-lab/bell5_jaco_kinect2.git
touch ~/bell5_jaco_kinect2/.on_aws

