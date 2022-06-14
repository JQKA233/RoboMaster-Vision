#include "kPredict.hpp"
#include <opencv2/core/types.hpp>

void kalmanFilter::Init(double s, double v, int Id)
    {
        switch (Id)
	{
	case 1:
		x_ << s, v;
        F1 << 1,1,0,1;
        P1 << 1,0,0,1;
        Q1 << 1,0,0,1;
        H1 << 1,0,0,1;
		break;
	case 2:
        y_ << s, v;
        F2 << 1,1,0,1;
        P2 << 1,0,0,1;
        Q2 << 1,0,0,1;
        H2 << 1,0,0,1;
		break;
	case 3:
        z_ << s, v;
        F3 << 1,1,0,1;
        P3 << 1,0,0,1;
        Q3 << 1,0,0,1;
        H3 << 1,0,0,1;
		break;
	default:
		cout << "WRONG ID GIVEN!" << endl;
		break;
	}
        

    }

void kalmanFilter::predict(int Id)
    {
        switch (Id)
        {
        case 1:
            // X' = FX + U
            x_ = F1 * x_;
            // P' = FPFT+Q
            P1 = F1 * P1 * F1.transpose() + Q1;
            break;
        case 2:
            y_ = F2 * y_;
            P2 = F2 * P2 * F2.transpose() + Q2;
            break;
        case 3:
            z_ = F3 * z_;
            P3 = F3 * P3 * F3.transpose() + Q3;
            break;
        default:
            cout << "WRONG ID GIVEN!" << endl;
            break;
        }
    }

double kalmanFilter::update(double m_s,double m_v,int Id){
        switch (Id)
        {
        case 1:
            {// Y = Z - HX'
            Eigen::Matrix<double,2,1> Z1 ;
            Z1 << m_s, m_v;
            Eigen::Matrix<double,2,1> Y1 = Z1 - H1 * x_;
            // S = HP'HT + R
            Eigen::Matrix<double,2,2> S1 = H1 * P1 * H1.transpose();
            // K = P'HTS-1
            K1 = P1 * H1.transpose() * S1.inverse();
            // X = X' + KY
            x_ = x_ + K1 * Y1;
            // P = (I - KH)P'
            Eigen::Matrix<double,2,2> I1 ;
            I1 << 1, 1, 1, 1;
            P1 = (I1 - K1 * H1) * P1;
            
            return x_(0, 0);
            }

            break;
        case 2:
            {// Y = Z - HX'
            Eigen::Matrix<double,2,1> Z2 ;
            Z2 << m_s, m_v;
            Eigen::Matrix<double,2,1> Y2 = Z2 - H2 * y_;
            // S = HP'HT + R
            Eigen::Matrix<double,2,2> S2 = H2 * P2 * H2.transpose();
            // K = P'HTS-1
            K2 = P2 * H2.transpose() * S2.inverse();
            // X = X' + KY
            y_ = y_ + K2 * Y2;
            // P = (I - KH)P'
            Eigen::Matrix<double,2,2> I2 ;
            I2 << 1, 1, 1, 1;
            P2 = (I2 - K2 * H2) * P2;
            
            return y_(0, 0);
            }
            break;
        case 3:
            {// Y = Z - HX'
            Eigen::Matrix<double,2,1> Z3 ;
            Z3 << m_s, m_v;
            Eigen::Matrix<double,2,1> Y3 = Z3 - H3 * z_;
            // S = HP'HT + R
            Eigen::Matrix<double,2,2> S3 = H3 * P3 * H3.transpose();
            // K = P'HTS-1
            K3 = P3 * H3.transpose() * S3.inverse();
            // X = X' + KY
            z_ = z_ + K3 * Y3;
            // P = (I - KH)P'
            Eigen::Matrix<double,2,2> I3 ;
            I3 << 1, 1, 1, 1;
            P3 = (I3 - K3 * H3) * P3;
        
            return z_(0, 0);
            }

            break;
        default:
            cout << "WRONG ID GIVEN!" << endl;
            break;
        }
        
    }

double kalmanFilter::getPredict(double x,int Id)
    {
        double x_pos,x_lastpos;
        double x_speed;
        double pre_s;
        x_speed = (x - x_lastpos);
        //cout<<"xspeed"<<x_speed<<endl;
        x_lastpos = x;
        //detect_flag == false;
        predict(Id);
        pre_s = update(x + x_speed*10, x_speed,Id);
        return pre_s;
    }