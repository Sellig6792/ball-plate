use std::env;

#[derive(Debug, Clone, Copy)]
pub enum Axe {
    X = 0,
    Y = 1,
}

pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub dt: f32, // Le dt est maintenant stocké directement dans la configuration
}

impl PidConfig {
    /// Charge les paramètres depuis le fichier .env et calcule le dt via le frame_rate
    pub fn from_env() -> Self {
        let kp: f32 = env::var("PID_KP")
            .expect("PID_KP manquant")
            .parse()
            .expect("PID_KP invalide");

        let ki: f32 = env::var("PID_KI")
            .expect("PID_KI manquant")
            .parse()
            .expect("PID_KI invalide");

        let kd: f32 = env::var("PID_KD")
            .expect("PID_KD manquant")
            .parse()
            .expect("PID_KD invalide");

        let fps: f32 = env::var("frame_rate")
            .expect("frame_rate manquant")
            .parse()
            .expect("frame_rate invalide");

        if fps <= 0.0 {
            panic!("Le frame_rate doit être strictement supérieur à 0");
        }

        Self {
            kp,
            ki,
            kd,
            dt: 1.0 / fps,
        }
    }
}

struct PidState {
    erreur_precedente: f32,
    somme_integrale: f32,
}

impl PidState {
    fn new() -> Self {
        Self {
            erreur_precedente: 0.0,
            somme_integrale: 0.0,
        }
    }
}

pub struct BallAndPlatePid {
    config: PidConfig,
    etat_x: PidState,
    etat_y: PidState,
}

impl BallAndPlatePid {
    /// Initialise le contrôleur en lisant directement les variables du fichier .env
    pub fn from_env() -> Self {
        let config = PidConfig::from_env();
        Self {
            config,
            etat_x: PidState::new(),
            etat_y: PidState::new(),
        }
    }

    /// Récupère le dt calculé (utile si vous en avez besoin ailleurs dans votre boucle OpenCV)
    pub fn get_dt(&self) -> f32 {
        self.config.dt
    }

    /// Calcule la commande d'inclinaison pour un axe (entre 0.2 et 0.8, plat à 0.5)
    pub fn calculer_inclinaison(&mut self, axe: Axe, position_balle: f32, consigne_centre: f32) -> f32 {
        let position = position_balle.clamp(0.0, 1.0);
        let dt = self.config.dt;

        let etat = match axe {
            Axe::X => &mut self.etat_x,
            Axe::Y => &mut self.etat_y,
        };

        let erreur = consigne_centre - position;

        // Proportionnel
        let p = self.config.kp * erreur;

        // Intégral
        if self.config.ki > 0.0 {
            etat.somme_integrale += erreur * dt;
            etat.somme_integrale = etat.somme_integrale.clamp(-0.2, 0.2);
        }
        let i = self.config.ki * etat.somme_integrale;

        // Dérivé
        let d = if dt > 0.0 {
            self.config.kd * ((erreur - etat.erreur_precedente) / dt)
        } else {
            0.0
        };

        etat.erreur_precedente = erreur;

        let commande_pid = p + i + d;
        let inclinaison_plaque = 0.5 + commande_pid;

        // Sécurité pour ne pas forcer sur les servos
        inclinaison_plaque.clamp(0.2, 0.8)
    }
}