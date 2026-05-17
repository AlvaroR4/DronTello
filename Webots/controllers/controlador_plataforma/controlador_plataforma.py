from controller import Supervisor

supervisor = Supervisor()
paso_simulacion = int(supervisor.getBasicTimeStep())

nodo_plataforma = supervisor.getFromDef("PLATAFORMA")
campo_posicion = nodo_plataforma.getField("translation")


tiempo_transcurrido = 0.0
tiempo_limite = 10.0         
velocidad_avance = 0.5       
altura_z = 0.3               

print("Iniciando ")

while supervisor.step(paso_simulacion) != -1:
    
    if tiempo_transcurrido < tiempo_limite:

        posicion_actual = campo_posicion.getSFVec3f()
        

        incremento_distancia = velocidad_avance * (paso_simulacion / 1000.0)
        

        nueva_posicion = [
            posicion_actual[0] + incremento_distancia, 
            posicion_actual[1],                     
            altura_z                           
        ]
        

        campo_posicion.setSFVec3f(nueva_posicion)
        
    elif tiempo_transcurrido >= tiempo_limite and tiempo_transcurrido < (tiempo_limite + (paso_simulacion/1000.0)):

        print("fin")
        

    tiempo_transcurrido += (paso_simulacion / 1000.0)
