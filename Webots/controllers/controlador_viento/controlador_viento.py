from controller import Supervisor

def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    dron_node = supervisor.getFromDef("Mavic_2_PRO")

    FUERZA_VIENTO = [0.0, 1.5, 0.0] 

    while supervisor.step(timestep) != -1:
        dron_node.addForce(FUERZA_VIENTO, False)

if __name__ == "__main__":
    main()