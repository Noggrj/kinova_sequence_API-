#include "KortexRobot.h"

int main(int argc, char **argv)
{   
    /*Conexão com o KinovaGen3*/
    KortexRobot Kinova = KortexRobot("192.168.1.10");

    /*Inicialização*/

    bool connected = Kinova.Init();

    
    if(connected)
    {
        std::cout<< "Conexão OK!" << std::endl;

        Kinova.SubscribeToNotifications();
        
        /* Algoritmo de manipulação para executar uma ação já existente, neste caso ação Home*/


        auto action_type = Kinova::Api::Base::RequestedActionType();
        action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
        Kinova.ExecuteExistingAction("Home" , action_type)
        Kinova.WaitWhileRobotIsMoving(100000);

        /* Algoritmo para mover para  uma pose cartesiana utilizando um vetor de poses e um de orientação*/

        Kinova::Api::Base::CartesianTrajectoryConstraint constraint = Kinova::Api::Base::CartesianTrajectoryConstraint();
        constraint.mutable_speed()-> set_translation(0.25f);
        constraint.mutable_speed()-> set_orientation(30.0f);

        Kinova.MoveTo(tCartesianVector(0.5f, 0.0f, 0.25f), tCartesian(140.0f, 0.0f, 0.0f), constraint);
        Kinova.WaitWhileRobotIsMoving(100000);

        /*Algoritmo para mover para um conjunto desejado de ângulos de junta */

        std::vector<float> jointPositions   {270.0f, 25.0f, 230.0f, 240.0f, 345.0f, 320.0f, 180.0f};
        Kinova.SetJointAngles(jointPositions, Kinova::Api::Base::JointTrajectoryConstraint());
        Kinova.WaitWhileRobotIsMoving(100000);


        /*Algoritmo para mover o Kinova usando comandos de torque, -> 10cm/s*/

        Kinova.SetTwistReferenceFrame(Kinova::Api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL);

        Kinova.SendTwistCommand(tCartesianVector(0.0f, 0.0f , -0.1f), tCartesianVector(0.0f, 0.0f , 0.0f));

        /* Obs: Já que os comandos de torção não são ações, não temos acesso a variável de controle, ocupado ou não,
        precisamos colocar para "dormir", manualmente. Uma desvantagem de usar os comandos de torque, */ 
        

        std:this_thread::sleep_for(std::chrono::milliseconds(2000));

        Kinova.stop()

        /* Algoritmo para comando de velocidade nas juntas*/

        std::vector<float> speeds (6, 0.0f)
        speeds.push_back(40.0f);
        Kinova.SetJointSpeeds(speeds);

        Kinova.stop()

        robot.UnsubscribeToNotifications();
    }
    std::cout<< "Fim da sequências de movimentos" <<std::endl;
}

