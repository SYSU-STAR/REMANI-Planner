#pragma once

#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>

#include <Eigen/Eigen>

#include "traj_utils/root_finder.hpp"
#include "mm_config/mm_config.hpp"

namespace poly_traj
{

    // Polynomial degree and trajectory dimension are fixed here
    // [TODO] template <int DIM_, int degree>
    template <int degree>
    class Piece
    {
    public:
        typedef Eigen::Matrix<double, 8, degree + 1> CoefficientMat;
        typedef Eigen::Matrix<double, 8, degree> VelCoefficientMat;
        typedef Eigen::Matrix<double, 8, degree - 1> AccCoefficientMat;

    private:
        double duration;
        CoefficientMat coeffMat;
        int dim = 8;
        int singul;
        double mobile_base_wheel_base = 0.370445;
        double mobile_base_wheel_radius = 0.07;
    public:
        Piece() = default;

        Piece(double dur, const CoefficientMat &cMat, int s)
            : duration(dur), coeffMat(cMat), singul(s) {}

        inline int getDim() const
        {
            return dim;
        }

        inline int getDegree() const
        {
            return degree;
        }

        inline double getDuration() const
        {
            return duration;
        }

        inline int getSingul() const
        {
            return singul;
        }

        inline const CoefficientMat &getCoeffMat() const
        {
            return coeffMat;
        }

        inline Eigen::VectorXd getPos(const double &t) const
        {
            Eigen::VectorXd pos(coeffMat.rows());
            pos.setZero();
            double tn = 1.0;
            for (int i = degree; i >= 0; i--)
            {
                pos += tn * coeffMat.col(i);
                tn *= t;
            }
            return pos;
        }

        inline Eigen::Matrix2d getCarR(const double &t) const
        {
            Eigen::Vector2d vel = getVel(t).head(2);
            Eigen::Matrix2d rotation_matrix;
            rotation_matrix << vel(0), -vel(1),
                               vel(1), vel(0);
            rotation_matrix = singul * rotation_matrix / vel.norm();

            return rotation_matrix;
        }

        inline Eigen::Matrix2d getCarRdot(const double &t) const
        {
            Eigen::Vector2d vel = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);
            Eigen::Matrix2d temp_a_ba, temp_v_bv;
            temp_a_ba << acc(0), -acc(1),
                         acc(1), acc(0);
            temp_v_bv << vel(0), -vel(1),
                         vel(1), vel(0);
            Eigen::Matrix2d R_dot = singul * (temp_a_ba / vel.norm() - temp_v_bv / pow(vel.norm(), 3) * (vel.transpose() * acc)); 
        
            return R_dot;
        }

        inline Eigen::VectorXd getVel(const double &t) const
        {
            Eigen::VectorXd vel(coeffMat.rows());
            vel.setZero();
            double tn = 1.0;
            int n = 1;
            for (int i = degree - 1; i >= 0; i--)
            {
                vel += n * tn * coeffMat.col(i);
                tn *= t;
                n++;
            }
            return vel;
        }

        inline Eigen::VectorXd getAcc(const double &t) const
        {
            Eigen::VectorXd acc(coeffMat.rows());
            acc.setZero();
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = degree - 2; i >= 0; i--)
            {
                acc += m * n * tn * coeffMat.col(i);
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        inline Eigen::VectorXd getJer(const double &t) const
        {
            Eigen::VectorXd jer(coeffMat.rows());
            jer.setZero();
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = degree - 3; i >= 0; i--)
            {
                jer += l * m * n * tn * coeffMat.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        inline double getCarAngle(const double &t) const
        {
            Eigen::Vector2d vel = getVel(t).head(2);
            return std::atan2(singul * vel(1), singul * vel(0)); //[-PI, PI]
        }

        inline double getCarVel(const double &t) const{
            Eigen::Vector2d vel  = getVel(t).head(2);
            return singul * vel.norm();
        }

        inline double getCarAcc(const double &t) const
        {
            Eigen::Vector2d vel  = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);

            if (vel.norm() < 1e-6){
              return 0.0;
            }else{
              return singul * (vel(0)*acc(0)+vel(1)*acc(1)) / vel.norm();
            }
        }

        inline double getCarOmega(const double &t) const
        {
            Eigen::Vector2d vel = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);

            if (vel.norm() < 1e-6){
              return 0.0;
            }else{
              return (vel(0) * acc(1) - vel(1) * acc(0)) / vel.squaredNorm();
            }
        }
        
        inline double getCarDomega(const double &t) const
        {
            Eigen::Vector2d vel  = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);
            Eigen::Vector2d jer = getJer(t).head(2);

            if (vel.norm() < 1e-6){
                return 0.0;
            }else{
                double domega = (vel(0)*jer(1)-vel(1)*jer(0)) / vel.squaredNorm();
                domega -= 2 * (vel(0) * acc(1) - vel(1) * acc(0)) * (vel(0) * acc(0) + vel(1) * acc(1)) / vel.squaredNorm() / vel.squaredNorm();
                return domega;
            }
        }

        inline double getCarLeftWheelOmega(const double &t) const
        {
            Eigen::Vector2d vel  = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);
            double v_norm = vel.norm();
            double vTv_inv = 1.0 / vel.squaredNorm(); // avoid siguality vel = 0
            Eigen::Matrix2d B_h;
            B_h << 0.0, -1.0,
                    1.0,  0.0;
            double aTBv = acc.transpose() * B_h * vel;
            double omega = aTBv * vTv_inv;
            double wheel_omega_left = (2.0 * singul * v_norm - mobile_base_wheel_base * omega) / (2 * mobile_base_wheel_radius);

            return wheel_omega_left;
        }

        inline double getCarRightWheelOmega(const double &t) const
        {
            Eigen::Vector2d vel  = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);
            double v_norm = vel.norm();
            double vTv_inv = 1.0 / vel.squaredNorm(); // avoid siguality vel = 0
            Eigen::Matrix2d B_h;
            B_h << 0.0, -1.0,
                    1.0,  0.0;
            double aTBv = acc.transpose() * B_h * vel;
            double omega = aTBv * vTv_inv;
            double wheel_omega_right = (2.0 * singul * v_norm + mobile_base_wheel_base * omega) / (2 * mobile_base_wheel_radius);

            return wheel_omega_right;
        }
        
        inline double getCarLeftWheelAlpha(const double &t) const
        {
            Eigen::Vector2d vel = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);
            Eigen::Vector2d jer = getJer(t).head(2);
            double v_norm = vel.norm();
            double vTv_inv = 1.0 / vel.squaredNorm(); // avoid siguality vel = 0
            double vTv_inv2 = vTv_inv * vTv_inv;
            Eigen::Matrix2d B_h;
            B_h << 0.0, -1.0,
                    1.0,  0.0;
            double aTv = acc.transpose() * vel;
            double aTBv = acc.transpose() * B_h * vel;
            double jTBv = jer.transpose() * B_h * vel;
            double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;
            double wheel_alpha_left = (2.0 * singul * aTv / v_norm - mobile_base_wheel_base * alpha) / (2 * mobile_base_wheel_radius);
            return wheel_alpha_left;
        }

        inline double getCarRightWheelAlpha(const double &t) const
        {
            Eigen::Vector2d vel = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);
            Eigen::Vector2d jer = getJer(t).head(2);
            double v_norm = vel.norm();
            double vTv_inv = 1.0 / vel.squaredNorm(); // avoid siguality vel = 0
            double vTv_inv2 = vTv_inv * vTv_inv;
            Eigen::Matrix2d B_h;
            B_h << 0.0, -1.0,
                    1.0,  0.0;
            double aTv = acc.transpose() * vel;
            double aTBv = acc.transpose() * B_h * vel;
            double jTBv = jer.transpose() * B_h * vel;
            double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;
            double wheel_alpha_right = (2.0 * singul * aTv / v_norm + mobile_base_wheel_base * alpha) / (2 * mobile_base_wheel_radius);
            return wheel_alpha_right;
        }

        inline Eigen::VectorXd getCarStateExpPos(const double &t) const
        {
            Eigen::Vector2d vel = getVel(t).head(2);
            Eigen::Vector2d acc = getAcc(t).head(2);
            Eigen::Vector2d jer = getJer(t).head(2);
           
            //state: angle, vel, acc, omega, domega
            Eigen::VectorXd CarState(5);


            // theta angle
            CarState[0] =  std::atan2(singul * vel(1), singul * vel(0));
            // vel
            CarState[1] = singul * vel.norm();


            if(abs(CarState[1]) < 1e-6){
                CarState[2] = 0.0;
                CarState[3] = 0.0;
                CarState[4] = 0.0;
            }else{
                // acc 
                CarState[2] =  singul * (vel(0) * acc(0) + vel(1) * acc(1)) / CarState[1];
                // omega
                CarState[3] =  (vel(0) * acc(1) - vel(1) * acc(0)) / std::pow(CarState[1], 2);
                // domega
                CarState[4] =  (vel(0)*jer(1)-vel(1)*jer(0)) / vel.squaredNorm();
                CarState[4] -= 2 * (vel(0)*acc(1)-vel(1)*acc(0)) * (vel(0)*acc(0)+vel(1)*acc(1)) / vel.squaredNorm() / vel.squaredNorm();
            }
            // std::cout << "singul is: "<<  singul << std::endl;
            // std::cout << "vel is: "<<  vel << std::endl;
            // std::cout << "acc is: "<< acc << std::endl;
            return CarState;
        }

        inline CoefficientMat normalizePosCoeffMat() const
        {
            CoefficientMat nPosCoeffsMat;
            double t = 1.0;
            for (int i = degree; i >= 0; i--)
            {
                nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
                t *= duration;
            }
            return nPosCoeffsMat;
        }

        inline VelCoefficientMat normalizeVelCoeffMat() const
        {
            VelCoefficientMat nVelCoeffMat;
            int n = 1;
            double t = duration;
            for (int i = degree - 1; i >= 0; i--)
            {
                nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
                t *= duration;
                n++;
            }
            return nVelCoeffMat;
        }

        inline AccCoefficientMat normalizeAccCoeffMat() const
        {
            AccCoefficientMat nAccCoeffMat;
            int n = 2;
            int m = 1;
            double t = duration * duration;
            for (int i = degree - 2; i >= 0; i--)
            {
                nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
                n++;
                m++;
                t *= duration;
            }
            return nAccCoeffMat;
        }

        inline double getMaxVelRate() const
        {
            VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getVel(0.0).norm();
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                        FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxVelRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                    it != candidates.end();
                    it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getVel((*it) * duration).squaredNorm();
                        maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                    }
                }
                return sqrt(maxVelRateSqr);
            }
        }

        inline double getMaxAccRate() const
        {
            AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return getAcc(0.0).norm();
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                        FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxAccRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                    it != candidates.end();
                    it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getAcc((*it) * duration).squaredNorm();
                        maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                    }
                }
                return sqrt(maxAccRateSqr);
            }
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            double sqrMaxVelRate = maxVelRate * maxVelRate;
            if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
                getVel(duration).squaredNorm() >= sqrMaxVelRate)
            {
                return false;
            }
            else
            {
                VelCoefficientMat nVelCoeffMat = normalizeVelCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(2));
                double t2 = duration * duration;
                coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            double sqrMaxAccRate = maxAccRate * maxAccRate;
            if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
                getAcc(duration).squaredNorm() >= sqrMaxAccRate)
            {
                return false;
            }
            else
            {
                AccCoefficientMat nAccCoeffMat = normalizeAccCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(2));
                double t2 = duration * duration;
                double t4 = t2 * t2;
                coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    template <int degree>
    class Trajectory
    {
    private:
        typedef std::vector<Piece<degree>> Pieces;
        Pieces pieces;
    public:
        Trajectory() = default;

        Trajectory(const std::vector<double> &durs,
                const std::vector<typename Piece<degree>::CoefficientMat> &cMats, 
                int s)
        {
            int N = std::min(durs.size(), cMats.size());
            pieces.reserve(N);
            for (int i = 0; i < N; i++)
            {
                pieces.emplace_back(durs[i], cMats[i], s);
            }
        }

        inline void GetState(double t, remani_planner::MMState &mm_state) const
        {
            double inner_t = t;
            if(inner_t > getTotalDuration())
                inner_t =getTotalDuration();
            int pieceIdx = locatePieceIdx(inner_t);
                       
            mm_state.car_pos = pieces[pieceIdx].getPos(inner_t).head(2);
            mm_state.car_vel = pieces[pieceIdx].getVel(inner_t).head(2);
            Eigen::VectorXd car_state = pieces[pieceIdx].getCarStateExpPos(inner_t);
            
            // angle, vel, acc, omega, domega
            mm_state.car_yaw    = car_state(0);  // heading angle
            mm_state.car_v      = car_state(1);
            mm_state.car_a      = car_state(2);
            mm_state.car_omega  = car_state(3);
            mm_state.car_domega = car_state(4);

            int traj_dim = getPieceDim();
            mm_state.mani_pos = pieces[pieceIdx].getPos(inner_t).head(traj_dim - 2);
            mm_state.mani_vel = pieces[pieceIdx].getVel(inner_t).head(traj_dim - 2);
        }

        inline int getPieceNum() const
        {
            return pieces.size();
        }

        inline int getPieceDim() const
        {
            return pieces.at(0).getDim();
        }

        inline Eigen::VectorXd getDurations() const
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        inline double getTotalDuration() const
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        inline Eigen::MatrixXd getPositions() const
        {
            int N = getPieceNum();
            int traj_dim = getPieceDim();
            Eigen::MatrixXd positions(traj_dim, N + 1);
            for (int i = 0; i < N; i++)
            {
                positions.col(i) = pieces[i].getCoeffMat().col(degree);
            }
            positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
            return positions;
        }

        inline const Piece<degree> &operator[](int i) const
        {
            return pieces[i];
        }

        inline Piece<degree> &operator[](int i)
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline typename Pieces::const_iterator begin() const
        {
            return pieces.begin();
        }

        inline typename Pieces::const_iterator end() const
        {
            return pieces.end();
        }

        inline typename Pieces::iterator begin()
        {
            return pieces.begin();
        }

        inline typename Pieces::iterator end()
        {
            return pieces.end();
        }

        inline void reserve(const int &n)
        {
            pieces.reserve(n);
            return;
        }

        inline void emplace_back(const Piece<degree> &piece)
        {
            pieces.emplace_back(piece);
            return;
        }

        inline void emplace_back(const double &dur,
                                const typename Piece<degree>::CoefficientMat &cMat,
                                int s)
        {
            pieces.emplace_back(dur, cMat, s);
            return;
        }

        inline void append(const Trajectory<degree> &traj)
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        inline int locatePieceIdx(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                idx < N &&
                t > (dur = pieces[idx].getDuration());
                idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            return idx;
        }

        inline Eigen::VectorXd getPos(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getPos(t);
        }

        inline Eigen::VectorXd getVel(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getVel(t);
        }

        inline Eigen::VectorXd getAcc(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getAcc(t);
        }

        inline Eigen::VectorXd getJer(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getJer(t);
        }

        inline Eigen::Matrix2d getCarR(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarR(t);
        }

        inline Eigen::Matrix2d getCarRdot(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarRdot(t);
        }

        inline double getCarAngle(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarAngle(t);
        }

        inline double getCarVel(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarVel(t);
        }

        inline double getCarAcc(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarAcc(t);
        }

        inline double getCarOmega(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarOmega(t);
        }

        inline double getCarDomega(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarDomega(t);
        }

        inline double getCarLeftWheelOmega(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarLeftWheelOmega(t);
        }

        inline double getCarRightWheelOmega(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarRightWheelOmega(t);
        }

        inline double getCarLeftWheelAlpha(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarLeftWheelAlpha(t);
        }

        inline double getCarRightWheelAlpha(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getCarRightWheelAlpha(t);
        }

        inline Eigen::VectorXd getJuncPos(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(degree);
            }
            else
            {
                return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::VectorXd getJuncVel(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(degree - 1);
            }
            else
            {
                return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::VectorXd getJuncAcc(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(degree - 2) * 2.0;
            }
            else
            {
                return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::VectorXd getJuncJerk(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(degree - 3) * 6.0;
            }
            else
            {
                return pieces[juncIdx - 1].getJer(pieces[juncIdx - 1].getDuration());
            }
        }

        inline std::pair<int, double> locatePieceIdxWithRatio(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            std::pair<int, double> idx_ratio;
            idx_ratio.first = idx;
            idx_ratio.second = t / dur;
            return idx_ratio;
        }

        inline Eigen::Vector2d getPoswithIdxRatio(double t, std::pair<int, double> &idx_ratio) const
        {
            idx_ratio = locatePieceIdxWithRatio(t);
            return pieces[idx_ratio.first].getPos(t);
        }

        inline double getMaxVelRate() const
        {
            int N = getPieceNum();
            double maxVelRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxVelRate();
                maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
            }
            return maxVelRate;
        }

        inline double getMaxAccRate() const
        {
            int N = getPieceNum();
            double maxAccRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxAccRate();
                maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
            }
            return maxAccRate;
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
            }
            return feasible;
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
            }
            return feasible;
        }
        
        inline Piece<degree> getPiece(int i) const
        {
            return pieces[i];
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solve(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solveAdj(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    template<int traj_dim_>
    class MinSnapOpt
    {
    public:
        MinSnapOpt() = default;
        ~MinSnapOpt() { A.destroy(); }

    private:
        int N; // pieceNum
        Eigen::MatrixXd headPVAJ, tailPVAJ;
        Eigen::MatrixXd b, c;
        BandedSystem A;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;
        Eigen::VectorXd T6;
        Eigen::VectorXd T7;
        // Eigen::Matrix<double, 6, 1> t, tInv;
        Eigen::MatrixXd gdC;
        // double gdT;
        /*MINCO descrips*/
        // Eigen::MatrixXd gdHead;
        // Eigen::MatrixXd gdTail;
        // Eigen::MatrixXd gdP;

    private:
        template <typename EIGENVEC>
        inline void addGradJbyT(EIGENVEC &gdT) const
        {
            // gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) += 576.0 * b.row(8 * i + 4).squaredNorm() +
                         5760.0 * b.row(8 * i + 4).dot(b.row(8 * i + 5)) * T1(i) +
                         14400.0 * b.row(8 * i + 5).squaredNorm() * T2(i) +
                         17280.0 * b.row(8 * i + 4).dot(b.row(8 * i + 6)) * T2(i) +
                         86400.0 * b.row(8 * i + 5).dot(b.row(8 * i + 6)) * T3(i) +
                         40320.0 * b.row(8 * i + 4).dot(b.row(8 * i + 7)) * T3(i) +
                         129600.0 * b.row(8 * i + 6).squaredNorm() * T4(i) +
                         201600.0 * b.row(8 * i + 5).dot(b.row(8 * i + 7)) * T4(i) +
                         604800.0 * b.row(8 * i + 6).dot(b.row(8 * i + 7)) * T5(i) +
                         705600.0 * b.row(8 * i + 7).squaredNorm() * T6(i);
            }
            return;
        }

        template <typename EIGENMAT>
        inline void addGradJbyC(EIGENMAT &gdC) const
        {
            for (int i = 0; i < N; i++)
            {
                gdC.row(8 * i + 7) += 10080.0 * b.row(8 * i + 4) * T4(i) +
                                     40320.0 * b.row(8 * i + 5) * T5(i) +
                                     100800.0 * b.row(8 * i + 6) * T6(i) +
                                     201600.0 * b.row(8 * i + 7) * T7(i);
                gdC.row(8 * i + 6) += 5760.0 * b.row(8 * i + 4) * T3(i) +
                                     21600.0 * b.row(8 * i + 5) * T4(i) +
                                     51840.0 * b.row(8 * i + 6) * T5(i) +
                                     100800.0 * b.row(8 * i + 7) * T6(i);
                gdC.row(8 * i + 5) += 2880.0 * b.row(8 * i + 4) * T2(i) +
                                     9600.0 * b.row(8 * i + 5) * T3(i) +
                                     21600.0 * b.row(8 * i + 6) * T4(i) +
                                     40320.0 * b.row(8 * i + 7) * T5(i);
                gdC.row(8 * i + 4) += 1152.0 * b.row(8 * i + 4) * T1(i) +
                                     2880.0 * b.row(8 * i + 5) * T2(i) +
                                     5760.0 * b.row(8 * i + 6) * T3(i) +
                                     10080.0 * b.row(8 * i + 7) * T4(i);
                // gdC.block<4, 3>(8 * i, 0).setZero();
            }
            return;
        }

        inline void solveAdjGradC(Eigen::MatrixXd &gdC) const
        {
            A.solveAdj(gdC);
            return;
        }

        template <typename EIGENVEC>
        inline void addPropCtoT(const Eigen::MatrixXd &adjGdC, EIGENVEC &gdT) const
        {
            Eigen::Matrix<double, 8, traj_dim_> B1;
            Eigen::Matrix<double, 4, traj_dim_> B2;
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(3) = -(b.row(i * 8 + 1) +
                              2.0 * T1(i) * b.row(i * 8 + 2) +
                              3.0 * T2(i) * b.row(i * 8 + 3) +
                              4.0 * T3(i) * b.row(i * 8 + 4) +
                              5.0 * T4(i) * b.row(i * 8 + 5) +
                              6.0 * T5(i) * b.row(i * 8 + 6) +
                              7.0 * T6(i) * b.row(i * 8 + 7));
                B1.row(4) = B1.row(3);

                // negative acceleration
                B1.row(5) = -(2.0 * b.row(i * 8 + 2) +
                              6.0 * T1(i) * b.row(i * 8 + 3) +
                              12.0 * T2(i) * b.row(i * 8 + 4) +
                              20.0 * T3(i) * b.row(i * 8 + 5) +
                              30.0 * T4(i) * b.row(i * 8 + 6) +
                              42.0 * T5(i) * b.row(i * 8 + 7));

                // negative jerk
                B1.row(6) = -(6.0 * b.row(i * 8 + 3) +
                              24.0 * T1(i) * b.row(i * 8 + 4) +
                              60.0 * T2(i) * b.row(i * 8 + 5) +
                              120.0 * T3(i) * b.row(i * 8 + 6) +
                              210.0 * T4(i) * b.row(i * 8 + 7));

                // negative snap
                B1.row(7) = -(24.0 * b.row(i * 8 + 4) +
                              120.0 * T1(i) * b.row(i * 8 + 5) +
                              360.0 * T2(i) * b.row(i * 8 + 6) +
                              840.0 * T3(i) * b.row(i * 8 + 7));

                // negative crackle
                B1.row(0) = -(120.0 * b.row(i * 8 + 5) +
                              720.0 * T1(i) * b.row(i * 8 + 6) +
                              2520.0 * T2(i) * b.row(i * 8 + 7));

                // negative d_crackle
                B1.row(1) = -(720.0 * b.row(i * 8 + 6) +
                              5040.0 * T1(i) * b.row(i * 8 + 7));

                // negative dd_crackle
                B1.row(2) = -5040.0 * b.row(i * 8 + 7);

                gdT(i) += B1.cwiseProduct(adjGdC.block<8, traj_dim_>(8 * i + 4, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(8 * N - 7) +
                          2.0 * T1(N - 1) * b.row(8 * N - 6) +
                          3.0 * T2(N - 1) * b.row(8 * N - 5) +
                          4.0 * T3(N - 1) * b.row(8 * N - 4) +
                          5.0 * T4(N - 1) * b.row(8 * N - 3) +
                          6.0 * T5(N - 1) * b.row(8 * N - 2) +
                          7.0 * T6(N - 1) * b.row(8 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(8 * N - 6) +
                          6.0 * T1(N - 1) * b.row(8 * N - 5) +
                          12.0 * T2(N - 1) * b.row(8 * N - 4) +
                          20.0 * T3(N - 1) * b.row(8 * N - 3) +
                          30.0 * T4(N - 1) * b.row(8 * N - 2) +
                          42.0 * T5(N - 1) * b.row(8 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(8 * N - 5) +
                          24.0 * T1(N - 1) * b.row(8 * N - 4) +
                          60.0 * T2(N - 1) * b.row(8 * N - 3) +
                          120.0 * T3(N - 1) * b.row(8 * N - 2) +
                          210.0 * T4(N - 1) * b.row(8 * N - 1));

            // negative snap
            B2.row(3) = -(24.0 * b.row(8 * N - 4) +
                          120.0 * T1(N - 1) * b.row(8 * N - 3) +
                          360.0 * T2(N - 1) * b.row(8 * N - 2) +
                          840.0 * T3(N - 1) * b.row(8 * N - 1));

            gdT(N - 1) += B2.cwiseProduct(adjGdC.block<4, traj_dim_>(8 * N - 4, 0)).sum();
            return;
        }

        template <typename EIGENMAT>
        inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP, Eigen::MatrixXd &gdHead, Eigen::MatrixXd &gdTail) const
        {
            for (int i = 0; i < N - 1; i++)
            {
                gdInP.col(i) = adjGdC.row(8 * i + 7).transpose(); 
            }
            gdHead = (adjGdC.topRows(4).transpose());
            gdTail = (adjGdC.bottomRows(4).transpose());
            // gdHead = (adjGdC.topRows(4).transpose()).topRows(2);
            // gdTail = (adjGdC.bottomRows(4).transpose()).topRows(2);
            return;
        }


    public:
        inline void reset(const int &pieceNum)
        {
            N = pieceNum;
            A.create(8 * N, 8, 8);
            b.resize(8 * N, traj_dim_);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            T6.resize(N);
            T7.resize(N);
            gdC.resize(8 * N, traj_dim_);
            // gdT.resize(6 * N);
            return;
        }

        inline void generate(const Eigen::MatrixXd &inPs,
                            const Eigen::VectorXd &ts,
                            const Eigen::MatrixXd &headState,
                            const Eigen::MatrixXd &tailState)
        {
            headPVAJ = headState;
            tailPVAJ = tailState;

            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);
            T6 = T4.cwiseProduct(T2);
            T7 = T4.cwiseProduct(T3);

            A.reset();
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            A(3, 3) = 6.0;
            b.row(0) = headPVAJ.col(0).transpose();
            b.row(1) = headPVAJ.col(1).transpose();
            b.row(2) = headPVAJ.col(2).transpose();
            b.row(3) = headPVAJ.col(3).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(8 * i + 4, 8 * i + 4) = 24.0;
                A(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
                A(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
                A(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
                A(8 * i + 4, 8 * i + 12) = -24.0;
                A(8 * i + 5, 8 * i + 5) = 120.0;
                A(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
                A(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
                A(8 * i + 5, 8 * i + 13) = -120.0;
                A(8 * i + 6, 8 * i + 6) = 720.0;
                A(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
                A(8 * i + 6, 8 * i + 14) = -720.0;
                A(8 * i + 7, 8 * i) = 1.0;
                A(8 * i + 7, 8 * i + 1) = T1(i);
                A(8 * i + 7, 8 * i + 2) = T2(i);
                A(8 * i + 7, 8 * i + 3) = T3(i);
                A(8 * i + 7, 8 * i + 4) = T4(i);
                A(8 * i + 7, 8 * i + 5) = T5(i);
                A(8 * i + 7, 8 * i + 6) = T6(i);
                A(8 * i + 7, 8 * i + 7) = T7(i);
                A(8 * i + 8, 8 * i) = 1.0;
                A(8 * i + 8, 8 * i + 1) = T1(i);
                A(8 * i + 8, 8 * i + 2) = T2(i);
                A(8 * i + 8, 8 * i + 3) = T3(i);
                A(8 * i + 8, 8 * i + 4) = T4(i);
                A(8 * i + 8, 8 * i + 5) = T5(i);
                A(8 * i + 8, 8 * i + 6) = T6(i);
                A(8 * i + 8, 8 * i + 7) = T7(i);
                A(8 * i + 8, 8 * i + 8) = -1.0;
                A(8 * i + 9, 8 * i + 1) = 1.0;
                A(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
                A(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
                A(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
                A(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
                A(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
                A(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
                A(8 * i + 9, 8 * i + 9) = -1.0;
                A(8 * i + 10, 8 * i + 2) = 2.0;
                A(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
                A(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
                A(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
                A(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
                A(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
                A(8 * i + 10, 8 * i + 10) = -2.0;
                A(8 * i + 11, 8 * i + 3) = 6.0;
                A(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
                A(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
                A(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
                A(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
                A(8 * i + 11, 8 * i + 11) = -6.0;

                b.row(8 * i + 7) = inPs.col(i).transpose();
            }

            A(8 * N - 4, 8 * N - 8) = 1.0;
            A(8 * N - 4, 8 * N - 7) = T1(N - 1);
            A(8 * N - 4, 8 * N - 6) = T2(N - 1);
            A(8 * N - 4, 8 * N - 5) = T3(N - 1);
            A(8 * N - 4, 8 * N - 4) = T4(N - 1);
            A(8 * N - 4, 8 * N - 3) = T5(N - 1);
            A(8 * N - 4, 8 * N - 2) = T6(N - 1);
            A(8 * N - 4, 8 * N - 1) = T7(N - 1);
            A(8 * N - 3, 8 * N - 7) = 1.0;
            A(8 * N - 3, 8 * N - 6) = 2.0 * T1(N - 1);
            A(8 * N - 3, 8 * N - 5) = 3.0 * T2(N - 1);
            A(8 * N - 3, 8 * N - 4) = 4.0 * T3(N - 1);
            A(8 * N - 3, 8 * N - 3) = 5.0 * T4(N - 1);
            A(8 * N - 3, 8 * N - 2) = 6.0 * T5(N - 1);
            A(8 * N - 3, 8 * N - 1) = 7.0 * T6(N - 1);
            A(8 * N - 2, 8 * N - 6) = 2.0;
            A(8 * N - 2, 8 * N - 5) = 6.0 * T1(N - 1);
            A(8 * N - 2, 8 * N - 4) = 12.0 * T2(N - 1);
            A(8 * N - 2, 8 * N - 3) = 20.0 * T3(N - 1);
            A(8 * N - 2, 8 * N - 2) = 30.0 * T4(N - 1);
            A(8 * N - 2, 8 * N - 1) = 42.0 * T5(N - 1);
            A(8 * N - 1, 8 * N - 5) = 6.0;
            A(8 * N - 1, 8 * N - 4) = 24.0 * T1(N - 1);
            A(8 * N - 1, 8 * N - 3) = 60.0 * T2(N - 1);
            A(8 * N - 1, 8 * N - 2) = 120.0 * T3(N - 1);
            A(8 * N - 1, 8 * N - 1) = 210.0 * T4(N - 1);

            b.row(8 * N - 4) = tailPVAJ.col(0).transpose();
            b.row(8 * N - 3) = tailPVAJ.col(1).transpose();
            b.row(8 * N - 2) = tailPVAJ.col(2).transpose();
            b.row(8 * N - 1) = tailPVAJ.col(3).transpose();

            A.factorizeLU();
            A.solve(b);
            
            return;
        }

        inline void reset(const Eigen::MatrixXd &headState,
                          const Eigen::MatrixXd &tailState,
                          const int &pieceNum)
        {
            N = pieceNum;
            headPVAJ = headState;
            tailPVAJ = tailState;
            // gdHead.resize(2, 4);
            // gdTail.resize(2, 4);
            A.create(8 * N, 8, 8);
            b.resize(8 * N, traj_dim_);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            T6.resize(N);
            T7.resize(N);
            gdC.resize(8 * N, traj_dim_);
            // gdT.resize(6 * N);
            return;
        }

        inline void generate(const Eigen::MatrixXd &inPs,
                             const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);
            T6 = T4.cwiseProduct(T2);
            T7 = T4.cwiseProduct(T3);

            A.reset();
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            A(3, 3) = 6.0;
            b.row(0) = headPVAJ.col(0).transpose();
            b.row(1) = headPVAJ.col(1).transpose();
            b.row(2) = headPVAJ.col(2).transpose();
            b.row(3) = headPVAJ.col(3).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(8 * i + 4, 8 * i + 4) = 24.0;
                A(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
                A(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
                A(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
                A(8 * i + 4, 8 * i + 12) = -24.0;
                A(8 * i + 5, 8 * i + 5) = 120.0;
                A(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
                A(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
                A(8 * i + 5, 8 * i + 13) = -120.0;
                A(8 * i + 6, 8 * i + 6) = 720.0;
                A(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
                A(8 * i + 6, 8 * i + 14) = -720.0;
                A(8 * i + 7, 8 * i) = 1.0;
                A(8 * i + 7, 8 * i + 1) = T1(i);
                A(8 * i + 7, 8 * i + 2) = T2(i);
                A(8 * i + 7, 8 * i + 3) = T3(i);
                A(8 * i + 7, 8 * i + 4) = T4(i);
                A(8 * i + 7, 8 * i + 5) = T5(i);
                A(8 * i + 7, 8 * i + 6) = T6(i);
                A(8 * i + 7, 8 * i + 7) = T7(i);
                A(8 * i + 8, 8 * i) = 1.0;
                A(8 * i + 8, 8 * i + 1) = T1(i);
                A(8 * i + 8, 8 * i + 2) = T2(i);
                A(8 * i + 8, 8 * i + 3) = T3(i);
                A(8 * i + 8, 8 * i + 4) = T4(i);
                A(8 * i + 8, 8 * i + 5) = T5(i);
                A(8 * i + 8, 8 * i + 6) = T6(i);
                A(8 * i + 8, 8 * i + 7) = T7(i);
                A(8 * i + 8, 8 * i + 8) = -1.0;
                A(8 * i + 9, 8 * i + 1) = 1.0;
                A(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
                A(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
                A(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
                A(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
                A(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
                A(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
                A(8 * i + 9, 8 * i + 9) = -1.0;
                A(8 * i + 10, 8 * i + 2) = 2.0;
                A(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
                A(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
                A(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
                A(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
                A(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
                A(8 * i + 10, 8 * i + 10) = -2.0;
                A(8 * i + 11, 8 * i + 3) = 6.0;
                A(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
                A(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
                A(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
                A(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
                A(8 * i + 11, 8 * i + 11) = -6.0;

                b.row(8 * i + 7) = inPs.col(i).transpose();
            }

            A(8 * N - 4, 8 * N - 8) = 1.0;
            A(8 * N - 4, 8 * N - 7) = T1(N - 1);
            A(8 * N - 4, 8 * N - 6) = T2(N - 1);
            A(8 * N - 4, 8 * N - 5) = T3(N - 1);
            A(8 * N - 4, 8 * N - 4) = T4(N - 1);
            A(8 * N - 4, 8 * N - 3) = T5(N - 1);
            A(8 * N - 4, 8 * N - 2) = T6(N - 1);
            A(8 * N - 4, 8 * N - 1) = T7(N - 1);
            A(8 * N - 3, 8 * N - 7) = 1.0;
            A(8 * N - 3, 8 * N - 6) = 2.0 * T1(N - 1);
            A(8 * N - 3, 8 * N - 5) = 3.0 * T2(N - 1);
            A(8 * N - 3, 8 * N - 4) = 4.0 * T3(N - 1);
            A(8 * N - 3, 8 * N - 3) = 5.0 * T4(N - 1);
            A(8 * N - 3, 8 * N - 2) = 6.0 * T5(N - 1);
            A(8 * N - 3, 8 * N - 1) = 7.0 * T6(N - 1);
            A(8 * N - 2, 8 * N - 6) = 2.0;
            A(8 * N - 2, 8 * N - 5) = 6.0 * T1(N - 1);
            A(8 * N - 2, 8 * N - 4) = 12.0 * T2(N - 1);
            A(8 * N - 2, 8 * N - 3) = 20.0 * T3(N - 1);
            A(8 * N - 2, 8 * N - 2) = 30.0 * T4(N - 1);
            A(8 * N - 2, 8 * N - 1) = 42.0 * T5(N - 1);
            A(8 * N - 1, 8 * N - 5) = 6.0;
            A(8 * N - 1, 8 * N - 4) = 24.0 * T1(N - 1);
            A(8 * N - 1, 8 * N - 3) = 60.0 * T2(N - 1);
            A(8 * N - 1, 8 * N - 2) = 120.0 * T3(N - 1);
            A(8 * N - 1, 8 * N - 1) = 210.0 * T4(N - 1);

            b.row(8 * N - 4) = tailPVAJ.col(0).transpose();
            b.row(8 * N - 3) = tailPVAJ.col(1).transpose();
            b.row(8 * N - 2) = tailPVAJ.col(2).transpose();
            b.row(8 * N - 1) = tailPVAJ.col(3).transpose();

            A.factorizeLU();
            A.solve(b);
            
            return;
        }

        inline const Eigen::MatrixXd &get_b() const
        {
            return b;
        }

        inline const Eigen::VectorXd &get_T1() const
        {
            return T1;
        }

        inline const Eigen::MatrixXd &getConstrainPoints() const
        {
            return getInitConstrainPoints(1);
        }

        inline Eigen::MatrixXd &get_gdC()
        {
            return gdC;
        }

        // inline  Eigen::MatrixXd  get_gdHead(void)
        // {
        //     return gdHead;
        // }

        // inline  Eigen::MatrixXd  get_gdTail(void)
        // {
        //     return gdTail;
        // }

        // inline Eigen::MatrixXd get_gdT() const
        // {
        //     return gdT;
        // }

        // inline Eigen::MatrixXd get_gdT(size_t i) const
        // {
        //     return gdT(i);
        // }

        inline double getTrajJerkCost() const
        {
            double energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 576.0 * b.row(8 * i + 4).squaredNorm() * T1(i) +
                          2880.0 * b.row(8 * i + 4).dot(b.row(8 * i + 5)) * T2(i) +
                          4800.0 * b.row(8 * i + 5).squaredNorm() * T3(i) +
                          5760.0 * b.row(8 * i + 4).dot(b.row(8 * i + 6)) * T3(i) +
                          21600.0 * b.row(8 * i + 5).dot(b.row(8 * i + 6)) * T4(i) +
                          10080.0 * b.row(8 * i + 4).dot(b.row(8 * i + 7)) * T4(i) +
                          25920.0 * b.row(8 * i + 6).squaredNorm() * T5(i) +
                          40320.0 * b.row(8 * i + 5).dot(b.row(8 * i + 7)) * T5(i) +
                          100800.0 * b.row(8 * i + 6).dot(b.row(8 * i + 7)) * T6(i) +
                          100800.0 * b.row(8 * i + 7).squaredNorm() * T7(i);
            }
            return energy;
        }

        inline Trajectory<7> getTraj(int s) const
        {
            Trajectory<7> traj;
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i),
                                  b.block<8, traj_dim_>(8 * i, 0)
                                      .transpose()
                                      .rowwise()
                                      .reverse(),
                                  s);
            }
            return traj;
        }

        inline Eigen::MatrixXd getInitConstrainPoints(const int K)
        {
            Eigen::MatrixXd pts(traj_dim_, N * K + 1);
            Eigen::VectorXd pos(traj_dim_);
            Eigen::Matrix<double, 8, 1> beta0;
            double s1, s2, s3, s4, s5, s6, s7;
            double step;
            int i_dp = 0;

            for (int i = 0; i < N; ++i)
            {
                const auto &c = b.block<8, traj_dim_>(i * 8, 0);
                step = T1(i) / K;
                s1 = 0.0;
                double t = 0;
                // innerLoop = K;

                for (int j = 0; j <= K; ++j)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    s6 = s3 * s3;
                    s7 = s6 * s1;
                    beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
                    pos = c.transpose() * beta0;
                    pts.col(i_dp) = pos;

                    s1 += step;
                    if (j != K || (j == K && i == N - 1))
                    {
                        ++i_dp;
                    }
                }
            }

            return pts;
        }

        template <typename EIGENVEC, typename EIGENMAT>
        inline void getGrad2TP(EIGENVEC &gdT,
                               EIGENMAT &gdInPs, 
                               Eigen::MatrixXd &gdHead, 
                               Eigen::MatrixXd &gdTail)
        {
            solveAdjGradC(gdC);
            addPropCtoT(gdC, gdT);
            addPropCtoP(gdC, gdInPs, gdHead, gdTail);
        }

        template <typename EIGENVEC>
        inline void initGradCost(EIGENVEC &gdT,
                                 double &cost)
        {
            // printf( "gdInPs=%d\n", gdInPs.size() );
            // gdT.resize(8*N);
            gdT.setZero();
            gdC.setZero();
            cost = getTrajJerkCost();
            addGradJbyT(gdT);
            addGradJbyC(gdC);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} //namespace poly_traj
