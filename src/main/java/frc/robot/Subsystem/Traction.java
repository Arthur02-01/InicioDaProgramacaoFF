package frc.robot.Subsystem;

// Imports da REV para trabalhar com os motores SparkMax,
// suas configurações, tipos de motor, modos de reset e persistência.
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// Import utilizado para pegar as informações dos encoders relativos
// presentes nos controladores Spark.
import com.revrobotics.RelativeEncoder;

// Import do giroscópio Pigeon2 da CTRE.
import com.ctre.phoenix6.hardware.Pigeon2;

// Imports da WPILib responsáveis pelo controle da tração diferencial,
// agrupamento dos motores e criação do subsistema.
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Importa as constantes do robô, principalmente os IDs dos motores.
import frc.robot.Constants;

/* Define a classe Traction como um subsistema da WPILib,
   utilizando o SubsystemBase como classe principal. */
public class Traction extends SubsystemBase {

    // VARIÁVEIS

    /* Variável utilizada para indicar se o modo turbo
       está ativado ou desativado. */
    public boolean turbo = false;


    // MOTORES

    /* Define o motor dianteiro direito.
       O ID é puxado da classe Constants e o motor foi definido
       como tipo kBrushed. */
    private SparkMax rightMotorFront =
            new SparkMax(
                Constants.TractionConstants.rightFrontMotorID,
                MotorType.kBrushed
            );

    /* Define o motor traseiro direito,
       também utilizando o ID configurado dentro de Constants. */
    private SparkMax rightMotorBack =
            new SparkMax(
                Constants.TractionConstants.rightBackMotorID,
                MotorType.kBrushed
            );

    /* Define o motor dianteiro esquerdo. */
    private SparkMax leftMotorFront =
            new SparkMax(
                Constants.TractionConstants.leftFrontMotorID,
                MotorType.kBrushed
            );

    /* Define o motor traseiro esquerdo. */
    private SparkMax leftMotorBack =
            new SparkMax(
                Constants.TractionConstants.leftBackMotorID,
                MotorType.kBrushed
            );


    // ENCODERS

    /* Cria as variáveis dos encoders.
       O encoder esquerdo será retirado do motor dianteiro esquerdo
       e o encoder direito será retirado do motor dianteiro direito. */
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;


    // GYRO

    /* Cria o giroscópio Pigeon2 utilizando o ID CAN 22.

       O final significa que, depois que esse objeto for criado,
       a referência dele não será alterada durante a execução. */
    private final Pigeon2 pigeon = new Pigeon2(22);


    // CONFIGURAÇÕES DOS SPARKS

    /* Cria duas configurações separadas:
       uma para os motores do lado esquerdo
       e outra para os motores do lado direito. */
    private SparkMaxConfig configSparkMotorEsquerda =
            new SparkMaxConfig();

    private SparkMaxConfig configSparkMotorDireita =
            new SparkMaxConfig();


    // GRUPOS DE MOTORES

    /* Agrupa os dois motores do lado esquerdo.

       Desta forma, quando o grupo esquerdo recebe uma velocidade,
       os dois motores recebem o comando juntos.

       O SuppressWarnings foi utilizado porque MotorControllerGroup
       apresenta aviso de remoção/depreciação. */
    @SuppressWarnings("removal")
    private MotorControllerGroup leftMotorControllerGroup =
            new MotorControllerGroup(
                leftMotorFront,
                leftMotorBack
            );

    /* Faz a mesma coisa para os dois motores do lado direito. */
    @SuppressWarnings("removal")
    private MotorControllerGroup rightMotorControllerGroup =
            new MotorControllerGroup(
                rightMotorFront,
                rightMotorBack
            );


    // DIFFERENTIAL DRIVE

    /* Cria o DifferentialDrive.

       Ele recebe um grupo de motores esquerdo
       e um grupo de motores direito.

       A partir disso a WPILib consegue calcular os comandos
       necessários para o robô andar para frente, para trás
       e realizar curvas. */
    private DifferentialDrive differentialDrive =
            new DifferentialDrive(
                leftMotorControllerGroup,
                rightMotorControllerGroup
            );


    // CONSTANTES FÍSICAS

    /* Define o diâmetro da roda em metros.

       0.1524 metros equivale a aproximadamente 6 polegadas. */
    private static final double WHEEL_DIAMETER_METERS = 0.1524;


    /* Calcula a distância percorrida pela roda em uma volta completa.

       Circunferência = PI × diâmetro.

       Portanto, essa variável representa quantos metros
       uma roda percorre em uma rotação completa. */
    private static final double METERS_PER_ROTATION =
            Math.PI * WHEEL_DIAMETER_METERS;


    // REDUÇÃO

    /* Define a relação de redução da transmissão.

       Neste caso está sendo utilizada uma redução de 10.71 para 1. */
    private static final double GEAR_RATIO = 10.71;


    // CONSTRUTOR

    public Traction() {

        /* Quando o subsistema Traction é criado,
           o giroscópio é resetado. */
        pigeon.reset();


        // CONFIGURAÇÃO DOS MOTORES DA DIREITA

        /* Configura os motores do lado direito.

           inverted(true):
           inverte o sentido dos motores.

           idleMode(kBrake):
           quando o motor não recebe potência,
           tenta permanecer parado.

           smartCurrentLimit(60):
           limita a corrente do controlador para 60 A. */
        configSparkMotorDireita
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);


        /* Aplica a configuração criada acima
           no motor dianteiro direito.

           kResetSafeParameters:
           reseta os parâmetros seguros antes de aplicar a configuração.

           kPersistParameters:
           salva as configurações no controlador. */
        rightMotorFront.configure(
                configSparkMotorDireita,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);


        /* Aplica a mesma configuração no motor traseiro direito. */
        rightMotorBack.configure(
                configSparkMotorDireita,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);


        // CONFIGURAÇÃO DOS MOTORES DA ESQUERDA

        /* Configura os motores do lado esquerdo.

           Neste caso inverted(false),
           portanto o lado esquerdo não está sendo invertido. */
        configSparkMotorEsquerda
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);


        /* Aplica a configuração ao motor dianteiro esquerdo. */
        leftMotorFront.configure(
                configSparkMotorEsquerda,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);


        /* Aplica a configuração ao motor traseiro esquerdo. */
        leftMotorBack.configure(
                configSparkMotorEsquerda,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);


        // ENCODERS

        /* Busca o encoder presente no Spark do motor dianteiro esquerdo
           e salva dentro da variável leftEncoder. */
        leftEncoder = leftMotorFront.getEncoder();


        /* Busca o encoder presente no Spark do motor dianteiro direito
           e salva dentro da variável rightEncoder. */
        rightEncoder = rightMotorFront.getEncoder();
    }


    // CONTROLE DE MOVIMENTO

    public void arcadeMode(double drive, double turn) {

        /* Controla o robô utilizando arcadeDrive.

           drive:
           controla o movimento para frente e para trás.

           turn:
           controla o giro do robô.

           O drive está sendo invertido utilizando -drive,
           enquanto turn está sendo mantido positivo. */
        differentialDrive.arcadeDrive(-drive, +turn);
    }


    public void stop() {

        /* Para os dois grupos de motores
           através do DifferentialDrive. */
        differentialDrive.stopMotor();
    }


    public void ativarTurbo(boolean turbo) {

        /* Recebe um valor booleano e salva dentro
           da variável turbo deste subsistema.

           this.turbo representa a variável da classe.

           turbo representa o valor recebido pela função. */
        this.turbo = turbo;
    }


    // GYRO

    public void resetYaw() {

        /* Reseta a leitura do Pigeon2. */
        pigeon.reset();
    }


    public double getYaw() {

        /* Retorna o ângulo registrado pelo Pigeon.

           O valor pode continuar aumentando ou diminuindo
           conforme o robô realiza múltiplas voltas. */
        return pigeon.getAngle();
    }


    // ENCODERS

    public void resetEncoders() {

        /* Zera a posição registrada
           pelo encoder do lado esquerdo. */
        leftEncoder.setPosition(0);

        /* Zera a posição registrada
           pelo encoder do lado direito. */
        rightEncoder.setPosition(0);
    }


    public double getAverageDistance() {

        /* Busca a quantidade de rotações registradas
           pelo encoder esquerdo.

           Divide pela relação de redução e depois multiplica
           pela distância percorrida por uma volta da roda.

           Dessa forma tenta transformar rotações do motor
           em metros percorridos pelo robô. */
        double leftDistance =
            (leftEncoder.getPosition() / GEAR_RATIO)
            * METERS_PER_ROTATION;


        /* Realiza o mesmo cálculo para o encoder direito.

           Entretanto, o valor do encoder direito
           está sendo multiplicado por -1. */
        double rightDistance =
            (-rightEncoder.getPosition() / GEAR_RATIO)
            * METERS_PER_ROTATION;


        /* Retorna o maior valor absoluto
           entre a distância esquerda e direita.

           Math.abs remove o sinal negativo.

           Math.max escolhe qual dos dois lados
           possui o maior valor. */
        return Math.max(
            Math.abs(leftDistance),
            Math.abs(rightDistance)
        );
    }
}